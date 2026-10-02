#!/usr/bin/env python3
"""
Standalone routing checker for status_server.

Mirrors the routing logic of TaskGenerator and robot_status_sync without
importing them, so it runs against a live database with no ROS environment
sourced and nothing running:

    TaskGenerator.create_adjacency_matrices  ->  build_matrices
    TaskGenerator.reconstruct_path           ->  reconstruct_path
    TaskGenerator.get_shortest_path          ->  shortest_path_between
    TaskGenerator._build_waypoints           ->  build_waypoints
    PostgresOperations.fetch_harvest_units   ->  fetch_sweep
    RobotStatusSync.get_closest_node         ->  snap

The mirroring is the point: if this script and the running node ever
disagree, one of them has drifted and the difference is the bug. Keep the
scipy call arguments and the guard conditions identical to the package.

Usage:
    python3 tools/path_check.py path 926 1
    python3 tools/path_check.py path 926 1 --waypoints
    python3 tools/path_check.py sweep --limit 80
    python3 tools/path_check.py task --at 970
    python3 tools/path_check.py snap -5.114 -0.895 1.600
    python3 tools/path_check.py edges 926
    python3 tools/path_check.py audit
    python3 tools/path_check.py audit --samples 20000

Every subcommand takes --config to point at a different config.yaml, and
--dbname to override the database named in it.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
from dataclasses import dataclass

try:
    import numpy as np
    import psycopg
    import yaml
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components, shortest_path
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    sys.exit(f'Missing dependency: {exc}. Needs numpy, scipy, psycopg and pyyaml.')


DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'config.yaml')

# Same sentinel scipy uses for an unreachable predecessor.
_SCIPY_SENTINEL = -9999

# Must match _MIN_LEG_M in task_generator.py.
MIN_LEG_M = 0.05

# A leg with more sideways than forward motion is one no planner will accept.
_LATERAL_FLAG = 0.05

# Filled from config.yaml at start-up; mirrors the `clearance` block.
CLEARANCE = {
    'enabled': True,
    'robot_half_width': 0.349,
    'robot_half_length': 0.495,
    'obstacle_radius': 0.225,
    'safety_margin': 0.05,
    'max_shift': 0.30,
}


# =============================================================================
# DATA
# =============================================================================


@dataclass
class Node:
    """A row of graph_node, or of farm_node when --source farm is used."""

    id: int
    node_type: str
    x: float
    y: float
    theta: float | None
    obj_id: int | None


@dataclass
class Unit:
    """One side of one bush: the smallest unit of harvesting work."""

    name: str
    node_id: int
    row_id: int
    x: float
    y: float


@dataclass
class Graph:
    """Everything the routing logic needs, loaded once."""

    nodes: dict[int, Node]
    edges: list[tuple[int, int, float, str]]
    cost_matrix: np.ndarray
    dist_matrix: np.ndarray
    predecessors: np.ndarray
    adjacency: dict[int, dict[int, tuple[float, str]]]
    objects: list[tuple[float, float]]


# =============================================================================
# DATABASE
# =============================================================================


def load_config(path: str) -> dict:
    """Read config.yaml, the same file the node reads."""
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def connect(config: dict, dbname_override: str | None):
    """Open a connection using the credentials in config.yaml."""
    db = config['database']
    dbname = dbname_override or db['dbname']

    conn = psycopg.connect(
        host=db['host'],
        port=db['port'],
        dbname=dbname,
        user=db['user'],
        password=db['password'],
        connect_timeout=db.get('connect_timeout', 5),
    )
    conn.autocommit = True
    return conn, dbname


def query(conn, sql: str, params: tuple = ()) -> list[tuple]:
    """Run a read-only query and return every row."""
    with conn.cursor() as cur:
        cur.execute(sql, params)
        return cur.fetchall()


def fetch_nodes(conn, source: str) -> dict[int, Node]:
    """Mirror of fetch_all_graph_nodes / fetch_all_nodes."""
    if source == 'graph':
        rows = query(conn, 'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node ORDER BY id ASC')
        return {int(r[0]): Node(int(r[0]), r[1], float(r[2]), float(r[3]),
                                None if r[4] is None else float(r[4]),
                                None if r[5] is None else int(r[5])) for r in rows}

    rows = query(conn, 'SELECT id, zone_prefix, x, y, type, theta FROM public.farm_node ORDER BY id ASC')
    return {int(r[0]): Node(int(r[0]), str(r[4]), float(r[2]), float(r[3]),
                            None if r[5] is None else float(r[5]), None) for r in rows}


def fetch_edges(conn, source: str) -> list[tuple[int, int, float, str]]:
    """Mirror of fetch_route_edges, keeping the edge type for reporting."""
    if source == 'graph':
        rows = query(conn, 'SELECT source_id, target_id, weight, edge_type FROM public.graph_edge')
        return [(int(r[0]), int(r[1]), float(r[2]), r[3]) for r in rows]

    rows = query(conn, 'SELECT from_node, to_node, distance FROM public.farm_edge')
    return [(int(r[0]), int(r[1]), float(r[2]), 'farm_edge') for r in rows]


def fetch_sweep(conn, source: str) -> list[Unit]:
    """Mirror of fetch_harvest_units: every bush side, already in sweep order."""
    if source != 'graph':
        rows = query(conn, 'SELECT id, name, x_map, y_map, conn_nodes FROM public.farm_asset_on_map ORDER BY id ASC')
        sweep = [Unit(r[1], min(r[4]), 0, float(r[2]), float(r[3])) for r in rows if r[4]]
        sweep += [Unit(r[1], max(r[4]), 0, float(r[2]), float(r[3])) for r in reversed(rows) if r[4] and len(r[4]) > 1]
        return sweep

    rows = query(conn, """
        WITH sides AS (
            SELECT o.object_id, o.row_id, o.x_coord, o.y_coord,
                   MIN(n.id) AS near_node, MAX(n.id) AS far_node
            FROM public.object_data o
            JOIN public.graph_node n ON n.obj_id = o.object_id
            WHERE n.node_type = 'Pickup'
            GROUP BY o.object_id, o.row_id, o.x_coord, o.y_coord
            HAVING COUNT(*) = 2
        )
        SELECT row_id, object_id, near_node AS node_id, x_coord, y_coord, 0 AS lane FROM sides
        UNION ALL
        SELECT row_id, object_id, far_node AS node_id, x_coord, y_coord, 1 AS lane FROM sides
        ORDER BY row_id ASC, lane ASC, node_id ASC
    """)

    return [Unit(f'b{r[0]}_{r[1]}', int(r[2]), int(r[0]), float(r[3]), float(r[4])) for r in rows]


def fetch_objects(conn, source: str) -> list[tuple[float, float]]:
    """Mirror of fetch_object_positions."""
    if source == 'graph':
        rows = query(conn, 'SELECT x_coord, y_coord FROM public.object_data')
    else:
        rows = query(conn, 'SELECT x_map, y_map FROM public.farm_asset_on_map')
    return [(float(r[0]), float(r[1])) for r in rows]


def fetch_done_pairs(conn) -> set[tuple[str, int]]:
    """Mirror of task_exists, fetched in one go rather than per bush side."""
    rows = query(conn, 'SELECT job_schedule, node_id FROM public.farm_harvesting_job')
    return {(r[0], int(r[1])) for r in rows}


# =============================================================================
# ROUTING - mirrors TaskGenerator
# =============================================================================


def build_matrices(nodes: dict[int, Node], edges: list[tuple[int, int, float, str]],
                   objects: list[tuple[float, float]] | None = None) -> Graph:
    """
    Mirror of TaskGenerator.create_adjacency_matrices.

    Two matrices over one sparsity pattern: cost from the stored weight,
    distance from the euclidean length of each edge. Edge weights are
    traversal costs and are two to four times the metres actually driven, so
    anything that means "how far" must read dist_matrix.
    """
    sources: list[int] = []
    targets: list[int] = []
    costs: list[float] = []
    lengths: list[float] = []
    adjacency: dict[int, dict[int, tuple[float, str]]] = {}
    skipped = 0

    for source, target, weight, edge_type in edges:
        start = nodes.get(source)
        end = nodes.get(target)

        if start is None or end is None:
            skipped += 1
            continue

        sources.append(source)
        targets.append(target)
        costs.append(weight)
        lengths.append(math.dist((start.x, start.y), (end.x, end.y)))
        adjacency.setdefault(source, {})[target] = (weight, edge_type)

    if skipped:
        print(f'  WARNING: {skipped} edges dropped, an endpoint is missing from the node table.')

    if not sources:
        sys.exit('No usable edges: every edge references a node that does not exist.')

    from_nodes = np.array(sources)
    to_nodes = np.array(targets)
    size = int(max(from_nodes.max(), to_nodes.max())) + 1
    shape = (size, size)

    cost_graph = coo_matrix((np.array(costs), (from_nodes, to_nodes)), shape=shape)
    distance_graph = coo_matrix((np.array(lengths), (from_nodes, to_nodes)), shape=shape)

    cost_matrix, predecessors = shortest_path(
        csgraph=cost_graph, method='auto', directed=True, return_predecessors=True)
    dist_matrix = shortest_path(
        csgraph=distance_graph, method='auto', directed=True, return_predecessors=False)

    return Graph(nodes, edges, cost_matrix, dist_matrix, predecessors, adjacency, objects or [])


def reconstruct_path(graph: Graph, start: int, end: int) -> list[int] | None:
    """
    Mirror of TaskGenerator.reconstruct_path, guards included.

    Walks the predecessor array backwards. The array records only real edges,
    so a returned path cannot contain a hop that is not in graph_edge - the
    audit subcommand proves that rather than assuming it.
    """
    size = graph.cost_matrix.shape[0]
    if not (0 <= start < size and 0 <= end < size):
        return None

    if np.isinf(graph.cost_matrix[start, end]):
        return None

    path: list[int] = []
    visited: set[int] = set()
    i = end

    while i != start:
        if i < 0:
            print(f'  reconstruct_path hit scipy sentinel ({_SCIPY_SENTINEL}) | start={start} end={end}')
            return None

        if i in visited:
            print(f'  reconstruct_path detected cycle at node {i} | start={start} end={end}')
            return None

        visited.add(i)
        path.append(int(i))
        i = int(graph.predecessors[start, i])

    path.append(start)
    return path[::-1]


def shortest_path_between(graph: Graph, start: int, end: int) -> list[int] | None:
    """Mirror of get_shortest_path, including the missing-node check."""
    route = reconstruct_path(graph, start, end)
    if not route:
        return None

    missing = [nid for nid in route if nid not in graph.nodes]
    if missing:
        print(f'  Route nodes missing from the node table: {missing}')
        return None

    return route


def build_waypoints(graph: Graph, route: list[int], raw: bool = False) -> list[tuple[int, float, float, float]]:
    """
    Mirror of TaskGenerator._build_waypoints.

    Pass raw=True to get the stored node headings straight from the table,
    which is what the node published before the collapse and re-orient passes
    were added. Comparing the two shows which legs used to be un-plannable.
    """
    count = len(route)
    poses: list[tuple[int, float, float, float]] = []

    for i, nid in enumerate(route):
        point = graph.nodes[nid]

        if point.theta is not None:
            theta = float(point.theta)
        else:
            if i > 0:
                previous = graph.nodes[route[i - 1]]
                dx = point.x - previous.x
                dy = point.y - previous.y
            elif i < count - 1:
                following = graph.nodes[route[i + 1]]
                dx = following.x - point.x
                dy = following.y - point.y
            else:
                dx, dy = 1.0, 0.0
            theta = math.atan2(dy, dx)

        poses.append((nid, point.x, point.y, theta))

    if raw:
        return poses

    collapsed: list[tuple[int, float, float, float]] = []
    for pose in poses:
        if collapsed and math.dist(collapsed[-1][1:3], pose[1:3]) < MIN_LEG_M:
            collapsed[-1] = pose
            continue
        collapsed.append(pose)

    collapsed = apply_clearance(graph, collapsed)

    out: list[tuple[int, float, float, float]] = []
    last = len(collapsed) - 1
    for i, (nid, x, y, theta) in enumerate(collapsed):
        if i < last:
            _, nx, ny, _ = collapsed[i + 1]
            heading = math.atan2(ny - y, nx - x)
        else:
            heading = theta
        out.append((nid, x, y, heading))

    return out


def apply_clearance(
    graph: Graph,
    poses: list[tuple[int, float, float, float]],
) -> list[tuple[int, float, float, float]]:
    """Mirror of TaskGenerator._apply_clearance."""
    if not CLEARANCE['enabled'] or not graph.objects:
        return poses

    standoff = CLEARANCE['obstacle_radius'] + CLEARANCE['robot_half_width'] + CLEARANCE['safety_margin']
    reach = CLEARANCE['robot_half_length'] + CLEARANCE['obstacle_radius']

    out = []
    for i, (nid, x, y, theta) in enumerate(poses):
        heading = travel_heading(poses, i, theta)
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        required, side = 0.0, 1.0
        for ox, oy in graph.objects:
            dx, dy = ox - x, oy - y
            along = dx * cos_h + dy * sin_h
            across = -dx * sin_h + dy * cos_h
            if abs(along) > reach:
                continue
            gap = standoff - abs(across)
            if gap > required:
                required, side = gap, (-1.0 if across > 0 else 1.0)

        if required <= 0.0:
            out.append((nid, x, y, theta))
            continue

        applied = min(required, CLEARANCE['max_shift'])
        px, py = -sin_h * side, cos_h * side
        out.append((nid, x + px * applied, y + py * applied, theta))

    return out


def travel_heading(poses, index: int, fallback: float) -> float:
    """Mirror of TaskGenerator._travel_heading."""
    _, x, y, _ = poses[index]
    if index + 1 < len(poses):
        _, nx, ny, _ = poses[index + 1]
    elif index > 0:
        _, px, py, _ = poses[index - 1]
        nx, ny = 2.0 * x - px, 2.0 * y - py
    else:
        return fallback
    if math.hypot(nx - x, ny - y) < 1e-6:
        return fallback
    return math.atan2(ny - y, nx - x)


def nearest_object(graph: Graph, x: float, y: float) -> tuple[float, float, float]:
    """Closest object to a point, as (x, y, distance)."""
    best = (0.0, 0.0, math.inf)
    for ox, oy in graph.objects:
        distance = math.hypot(x - ox, y - oy)
        if distance < best[2]:
            best = (ox, oy, distance)
    return best


def lateral_offset(a: tuple[int, float, float, float], b: tuple[int, float, float, float]) -> tuple[float, float]:
    """Forward and sideways components of a leg, in the frame of its start pose."""
    dx, dy = b[1] - a[1], b[2] - a[2]
    heading = a[3]
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    return forward, lateral


def snap(graph: Graph, x: float, y: float, yaw: float) -> tuple[int, bool]:
    """
    Mirror of RobotStatusSync.get_closest_node.

    Position alone cannot separate the two one-way lanes of a row - they are
    under a metre apart - so candidates are filtered to those facing within 90
    degrees of the robot before the nearest is taken.

    Returns:
        (node id, whether the heading filter was applied).
    """
    navigable = [n for n in graph.nodes.values() if n.id in graph.adjacency or any(
        n.id in targets for targets in graph.adjacency.values())]

    def facing_same_way(node: Node) -> bool:
        if node.theta is None:
            return True
        difference = math.atan2(math.sin(node.theta - yaw), math.cos(node.theta - yaw))
        return abs(difference) < math.pi / 2

    aligned = [n for n in navigable if facing_same_way(n)]
    used_heading = bool(aligned)

    if not aligned:
        aligned = navigable

    closest = min(aligned, key=lambda n: math.hypot(n.x - x, n.y - y))
    return closest.id, used_heading


# =============================================================================
# REPORTING
# =============================================================================


def describe_hops(graph: Graph, route: list[int]) -> bool:
    """Print each hop with its edge, and say whether every hop is real."""
    all_real = True
    total_cost = 0.0
    total_metres = 0.0

    print(f'  {"from":>6} {"to":>6}  {"weight":>10} {"metres":>8}  edge_type')
    for a, b in zip(route, route[1:]):
        edge = graph.adjacency.get(a, {}).get(b)
        metres = math.dist((graph.nodes[a].x, graph.nodes[a].y), (graph.nodes[b].x, graph.nodes[b].y))
        total_metres += metres

        if edge is None:
            all_real = False
            print(f'  {a:>6} {b:>6}  {"-":>10} {metres:>8.3f}  *** NO EDGE ***')
        else:
            total_cost += edge[0]
            print(f'  {a:>6} {b:>6}  {edge[0]:>10.3f} {metres:>8.3f}  {edge[1]}')

    print(f'  {"":>6} {"":>6}  {total_cost:>10.3f} {total_metres:>8.3f}  total')
    return all_real


# =============================================================================
# SUBCOMMANDS
# =============================================================================


def cmd_path(graph: Graph, conn, args) -> int:
    """Route between two nodes, hop by hop."""
    start, end = args.start, args.end

    for nid in (start, end):
        if nid not in graph.nodes:
            print(f'Node {nid} is not in the node table.')
            return 1

    route = shortest_path_between(graph, start, end)

    if route is None:
        print(f'No route from {start} to {end}.')
        return 1

    print(f'Route {start} -> {end}: {len(route)} nodes')
    print(f'  {route}')
    print()
    all_real = describe_hops(graph, route)
    print()
    print(f'  cost_matrix[{start},{end}] = {graph.cost_matrix[start, end]:.3f}   (routing cost, NOT metres)')
    print(f'  dist_matrix[{start},{end}] = {graph.dist_matrix[start, end]:.3f} m (what the battery check reads)')

    if not all_real:
        print()
        print('  FAIL: route contains a hop that is not an edge.')
        return 1

    if args.waypoints:
        print()
        report_waypoints(graph, route)

    return 0


def report_waypoints(graph: Graph, route: list[int]) -> int:
    """Print raw versus published waypoints and flag any un-drivable leg."""
    raw = build_waypoints(graph, route, raw=True)
    out = build_waypoints(graph, route)

    print(f'Raw node headings ({len(raw)} waypoints) - what the table stores:')
    bad = 0
    for i, pose in enumerate(raw):
        nid, x, y, theta = pose
        note = ''
        if i < len(raw) - 1:
            forward, lateral = lateral_offset(pose, raw[i + 1])
            if math.dist(pose[1:3], raw[i + 1][1:3]) < MIN_LEG_M:
                note = '  <-- ZERO-LENGTH LEG'
                bad += 1
            elif abs(lateral) > _LATERAL_FLAG and abs(lateral) > abs(forward):
                note = f'  <-- SIDEWAYS LEG forward={forward:+.3f} lateral={lateral:+.3f}'
                bad += 1
        print(f'  node_id={nid:<4} x={x:>9.4f} y={y:>9.4f} theta={theta:>7.4f}  '
              f'({graph.nodes[nid].node_type}){note}')

    print()
    print(f'Published waypoints ({len(out)}) - after collapse and re-orient:')
    still_bad = 0
    for i, pose in enumerate(out):
        nid, x, y, theta = pose
        note = ''
        if i < len(out) - 1:
            forward, lateral = lateral_offset(pose, out[i + 1])
            if abs(lateral) > _LATERAL_FLAG and abs(lateral) > abs(forward):
                note = f'  <-- STILL SIDEWAYS forward={forward:+.3f} lateral={lateral:+.3f}'
                still_bad += 1
        print(f'  node_id={nid:<4} x={x:>9.4f} y={y:>9.4f} theta={theta:>7.4f}  '
              f'({graph.nodes[nid].node_type}){note}')

    print()
    print(f'  legs Nav2 would reject before the fix: {bad}')
    print(f'  legs Nav2 would reject after the fix : {still_bad}')
    if len(out) != len(raw):
        print(f'  waypoints collapsed: {len(raw) - len(out)}')

    return still_bad


def cmd_sweep(graph: Graph, conn, args) -> int:
    """Print the harvest sweep in the order tasks will be generated."""
    sweep = fetch_sweep(conn, args.source)
    done = fetch_done_pairs(conn)

    if not sweep:
        print('Sweep is empty.')
        return 1

    print(f'Sweep: {len(sweep)} units, {len(done)} task rows already written')
    print()

    shown = sweep if args.limit == 0 else sweep[:args.limit]
    for i, unit in enumerate(shown, start=1):
        marker = 'done' if (unit.name, unit.node_id) in done else ''
        missing = '' if unit.node_id in graph.nodes else '  *** NODE MISSING ***'
        print(f'  {i:>4}. {unit.name:<12} node={unit.node_id:<5} row={unit.row_id:<3} '
              f'({unit.x:>8.3f}, {unit.y:>8.3f}) {marker}{missing}')

    if args.limit and len(sweep) > args.limit:
        print(f'  ... {len(sweep) - args.limit} more (use --limit 0 for all)')

    orphans = [u for u in sweep if u.node_id not in graph.nodes]
    if orphans:
        print()
        print(f'  FAIL: {len(orphans)} units point at a node that does not exist.')
        return 1

    return 0


def cmd_task(graph: Graph, conn, args) -> int:
    """Simulate the next harvest task from a given position."""
    sweep = fetch_sweep(conn, args.source)
    done = fetch_done_pairs(conn)

    target = next((u for u in sweep if (u.name, u.node_id) not in done), None)

    if target is None:
        print('No unharvested bush sides remain.')
        return 1

    print(f'Next harvest target: {target.name} via node {target.node_id} '
          f'(bush at {target.x:.3f}, {target.y:.3f})')
    print()

    route = shortest_path_between(graph, args.at, target.node_id)

    if route is None:
        print(f'FAIL: no route from node {args.at} to node {target.node_id}. Task would return None.')
        return 1

    print(f'Route {args.at} -> {target.node_id}: {len(route)} nodes')
    print(f'  {route}')
    print()
    all_real = describe_hops(graph, route)
    print()
    still_bad = report_waypoints(graph, route)

    return 0 if all_real and not still_bad else 1


def cmd_snap(graph: Graph, conn, args) -> int:
    """Show which node a pose snaps to, with and without the heading filter."""
    node_id, used_heading = snap(graph, args.x, args.y, args.yaw)
    node = graph.nodes[node_id]

    position_only = min(graph.nodes.values(), key=lambda n: math.hypot(n.x - args.x, n.y - args.y))

    print(f'Pose ({args.x:.3f}, {args.y:.3f}) yaw {args.yaw:.4f} rad')
    print()
    print(f'  with heading filter : node {node_id:<5} ({node.x:.3f}, {node.y:.3f}) '
          f'theta={node.theta if node.theta is None else round(node.theta, 4)} '
          f'type={node.node_type} dist={math.hypot(node.x - args.x, node.y - args.y):.3f} m')
    print(f'  position only       : node {position_only.id:<5} ({position_only.x:.3f}, {position_only.y:.3f}) '
          f'theta={position_only.theta if position_only.theta is None else round(position_only.theta, 4)} '
          f'type={position_only.node_type} dist={math.hypot(position_only.x - args.x, position_only.y - args.y):.3f} m')
    print()

    if not used_heading:
        print('  Heading filter matched nothing; fell back to position. Robot is broadside to every node.')
    elif node_id != position_only.id:
        print('  Heading filter CHANGED the result. Without it the robot would snap to the opposite lane.')
    else:
        print('  Heading filter agrees with position alone here.')

    return 0


def cmd_edges(graph: Graph, conn, args) -> int:
    """List every edge into and out of one node."""
    node = graph.nodes.get(args.node)
    if node is None:
        print(f'Node {args.node} is not in the node table.')
        return 1

    print(f'Node {node.id}: type={node.node_type} ({node.x:.3f}, {node.y:.3f}) '
          f'theta={node.theta if node.theta is None else round(node.theta, 4)} obj_id={node.obj_id}')
    print()

    outgoing = sorted(graph.adjacency.get(node.id, {}).items())
    incoming = sorted((src, data[node.id]) for src, data in graph.adjacency.items() if node.id in data)

    print(f'  OUT ({len(outgoing)}):')
    for target, (weight, edge_type) in outgoing:
        metres = math.dist((node.x, node.y), (graph.nodes[target].x, graph.nodes[target].y))
        print(f'    {node.id} -> {target:<5} weight={weight:>9.3f} metres={metres:>7.3f}  {edge_type}')

    print(f'  IN  ({len(incoming)}):')
    for source, (weight, edge_type) in incoming:
        metres = math.dist((graph.nodes[source].x, graph.nodes[source].y), (node.x, node.y))
        print(f'    {source:<5} -> {node.id} weight={weight:>9.3f} metres={metres:>7.3f}  {edge_type}')

    return 0


def cmd_audit(graph: Graph, conn, args) -> int:
    """Check graph health and prove that generated paths only use real edges."""
    failures = 0

    size = graph.cost_matrix.shape[0]
    print(f'Nodes: {len(graph.nodes)}   Edges: {len(graph.edges)}   Matrix: {size} x {size}')

    node_ids = sorted(graph.nodes)
    print(f'Node IDs: {node_ids[0]} .. {node_ids[-1]}')
    if node_ids[0] == 0:
        print('  NOTE: node id 0 exists; the matrix normally has a phantom row 0.')

    dangling = [(a, b) for a, b, _, _ in graph.edges if a not in graph.nodes or b not in graph.nodes]
    if dangling:
        failures += 1
        print(f'  FAIL: {len(dangling)} edges reference a node that does not exist, e.g. {dangling[:5]}')
    else:
        print('  OK: every edge endpoint exists.')

    isolated = [n for n in graph.nodes
                if n not in graph.adjacency and not any(n in t for t in graph.adjacency.values())]
    if isolated:
        print(f'  WARNING: {len(isolated)} nodes have no edges: {isolated[:10]}')
        print('           A robot snapped to one of these can never be routed.')
    else:
        print('  OK: every node has at least one edge.')

    real_ids = np.array(node_ids)
    sub = graph.cost_matrix[np.ix_(real_ids, real_ids)]
    unreachable = int(np.isinf(sub).sum()) - len(real_ids) * 0
    unreachable -= int(np.isinf(np.diag(sub)).sum())
    if unreachable:
        failures += 1
        print(f'  FAIL: {unreachable} unreachable ordered pairs out of {sub.size - len(real_ids)}.')
    else:
        print(f'  OK: all {sub.size - len(real_ids)} ordered node pairs are reachable.')

    components, _ = connected_components(
        coo_matrix((np.ones(len(graph.edges)),
                    (np.array([e[0] for e in graph.edges]), np.array([e[1] for e in graph.edges]))),
                   shape=(size, size)),
        directed=True, connection='strong')
    print(f'  Strongly connected components: {components} '
          f'(1 real component plus any phantom rows the matrix pads with)')

    zero_weight = [(a, b, t) for a, b, w, t in graph.edges if w == 0.0]
    if zero_weight:
        print(f'  NOTE: {len(zero_weight)} zero-weight edges, e.g. {zero_weight[0][2]}. '
              'Free to traverse, and legal.')

    print()
    print(f'Path audit: {args.samples} random ordered pairs')
    random.seed(args.seed)
    tested = 0
    bad: list[tuple[int, int, int, int]] = []

    for _ in range(args.samples):
        start = random.choice(node_ids)
        end = random.choice(node_ids)
        if start == end:
            continue

        route = reconstruct_path(graph, start, end)
        if route is None:
            continue

        tested += 1
        for a, b in zip(route, route[1:]):
            if b not in graph.adjacency.get(a, {}):
                bad.append((start, end, a, b))
                break

    if bad:
        failures += 1
        print(f'  FAIL: {len(bad)} of {tested} paths contain a hop that is not an edge.')
        for start, end, a, b in bad[:10]:
            print(f'    {start} -> {end} used non-edge {a} -> {b}')
    else:
        print(f'  OK: {tested} paths, every hop is a real edge.')

    print()
    print('Cost vs distance (edge weights are costs, not metres):')
    by_type: dict[str, list[tuple[float, float]]] = {}
    for a, b, w, t in graph.edges:
        if a in graph.nodes and b in graph.nodes:
            metres = math.dist((graph.nodes[a].x, graph.nodes[a].y), (graph.nodes[b].x, graph.nodes[b].y))
            by_type.setdefault(t, []).append((w, metres))

    print(f'  {"edge_type":<16} {"count":>6} {"avg weight":>11} {"avg metres":>11} {"ratio":>7}')
    for edge_type in sorted(by_type):
        pairs = by_type[edge_type]
        avg_w = sum(p[0] for p in pairs) / len(pairs)
        avg_m = sum(p[1] for p in pairs) / len(pairs)
        ratio = f'{avg_w / avg_m:.2f}' if avg_m > 1e-9 else '-'
        print(f'  {edge_type:<16} {len(pairs):>6} {avg_w:>11.3f} {avg_m:>11.3f} {ratio:>7}')

    print()
    print('FAILURES: ' + ('none' if failures == 0 else str(failures)))
    return 1 if failures else 0


# =============================================================================
# ENTRY POINT
# =============================================================================


def main() -> int:
    """Parse arguments, load the graph once, dispatch to the subcommand."""
    parser = argparse.ArgumentParser(
        description='Standalone routing checker mirroring status_server. Read-only.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Usage:')[1] if 'Usage:' in __doc__ else None,
    )
    parser.add_argument('--config', default=DEFAULT_CONFIG, help='path to config.yaml')
    parser.add_argument('--dbname', default=None, help='override the database named in config.yaml')
    parser.add_argument('--source', default=None, choices=['graph', 'farm'],
                        help='table family to read; defaults to farm.graph_source in config.yaml')

    sub = parser.add_subparsers(dest='command', required=True)

    p_path = sub.add_parser('path', help='route between two nodes, hop by hop')
    p_path.add_argument('start', type=int)
    p_path.add_argument('end', type=int)
    p_path.add_argument('--waypoints', action='store_true', help='also print the WayPoint list')
    p_path.set_defaults(func=cmd_path)

    p_sweep = sub.add_parser('sweep', help='harvest order, as tasks will be generated')
    p_sweep.add_argument('--limit', type=int, default=40, help='units to show, 0 for all')
    p_sweep.set_defaults(func=cmd_sweep)

    p_task = sub.add_parser('task', help='simulate the next harvest task from a position')
    p_task.add_argument('--at', type=int, required=True, help='node the robot currently occupies')
    p_task.set_defaults(func=cmd_task)

    p_snap = sub.add_parser('snap', help='which node a pose snaps to')
    p_snap.add_argument('x', type=float)
    p_snap.add_argument('y', type=float)
    p_snap.add_argument('yaw', type=float, help='heading in radians')
    p_snap.set_defaults(func=cmd_snap)

    p_edges = sub.add_parser('edges', help='every edge into and out of one node')
    p_edges.add_argument('node', type=int)
    p_edges.set_defaults(func=cmd_edges)

    p_audit = sub.add_parser('audit', help='graph health and path validity')
    p_audit.add_argument('--samples', type=int, default=5000)
    p_audit.add_argument('--seed', type=int, default=0)
    p_audit.set_defaults(func=cmd_audit)

    args = parser.parse_args()

    config = load_config(args.config)
    source = args.source or config.get('farm', {}).get('graph_source', 'graph')
    args.source = source

    conn, dbname = connect(config, args.dbname)
    print(f'Database: {dbname}   graph_source: {source}')
    print()

    try:
        nodes = fetch_nodes(conn, source)
        edges = fetch_edges(conn, source)

        if not nodes or not edges:
            sys.exit('Node or edge table is empty.')

        objects = fetch_objects(conn, source)

        block = config.get('clearance') or {}
        for key in CLEARANCE:
            if key in block:
                CLEARANCE[key] = block[key]

        graph = build_matrices(nodes, edges, objects)
        return args.func(graph, conn, args)
    finally:
        conn.close()


if __name__ == '__main__':
    sys.exit(main())

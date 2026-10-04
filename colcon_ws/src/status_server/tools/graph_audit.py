#!/usr/bin/env python3
"""
Grade a topological graph on whether a real robot can drive it.

path_check.py answers "is this route connected". This answers a different
question: "is this graph shaped so a planner can actually execute it". A graph
can be perfectly connected and still be undrivable.

Every check is geometric and offline. Nothing here talks to ROS or Nav2, so a
graph can be graded the moment it is generated, before anything is deployed.

Checks
    sideways   Edges asking the robot to move mostly perpendicular to the
               heading it holds at the start node. Nav2 aborts these. Usually
               means an approach node is missing between the two.
    zerolen    Edges between nodes at the same coordinates. Zero-length goals.
    approach   Entering/Exiting nodes reachable only by a sideways edge, with
               the node that should be added to fix each one.
    turns      Heading change demanded at each node, and whether the robot's
               turning radius fits the space it has to turn in.
    clearance  Node to nearest object, against footprint and inflation.
    dubins     Whether a car-like planner can join each edge's two poses.

Usage
    python3 tools/graph_audit.py
    python3 tools/graph_audit.py --checks sideways,approach
    python3 tools/graph_audit.py --sql
    python3 tools/graph_audit.py --robot-width 0.698 --inflation 0.4
    python3 tools/graph_audit.py --limit 0

Exit code is the number of failed checks, so it drops straight into CI.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field

try:
    import psycopg
    import yaml
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    sys.exit(f'Missing dependency: {exc}. Needs psycopg and pyyaml.')

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'config.yaml')

ALL_CHECKS = ['sideways', 'zerolen', 'approach', 'turns', 'clearance', 'dubins']

# A300 defaults. Override on the command line for a different robot.
ROBOT_LENGTH = 0.990
ROBOT_WIDTH = 0.698
TURNING_RADIUS = 0.200
INFLATION_RADIUS = 0.800
COST_SCALING = 4.0

# A leg is "sideways" when the across-track component beats the along-track one
# by this margin. Small offsets are just node placement noise.
LATERAL_TOL = 0.05

# Coordinates closer than this are the same place.
SAME_PLACE_M = 0.05


# =============================================================================
# DATA
# =============================================================================


@dataclass
class Node:
    """A row of graph_node."""

    id: int
    node_type: str
    x: float
    y: float
    theta: float | None
    obj_id: int | None


@dataclass
class Edge:
    """A row of graph_edge, with the geometry the checks need."""

    source: int
    target: int
    weight: float
    edge_type: str
    length: float = 0.0
    forward: float = 0.0
    lateral: float = 0.0
    dtheta: float = 0.0


@dataclass
class Robot:
    """The shape and steering limits the graph is being graded against."""

    length: float = ROBOT_LENGTH
    width: float = ROBOT_WIDTH
    turning_radius: float = TURNING_RADIUS
    inflation_radius: float = INFLATION_RADIUS
    cost_scaling: float = COST_SCALING

    @property
    def inscribed(self) -> float:
        """Largest circle that fits inside the footprint."""
        return self.width / 2.0

    @property
    def circumscribed(self) -> float:
        """Smallest circle that contains the footprint."""
        return math.hypot(self.length / 2.0, self.width / 2.0)

    @property
    def swept(self) -> float:
        """
        Furthest any footprint corner gets from the turn centre.

        The turn centre sits `turning_radius` to the side of the robot origin,
        so a tight radius does not make the robot sweep less - it makes it
        sweep more, because the body pivots around a point inside itself.
        """
        half_l, half_w = self.length / 2.0, self.width / 2.0
        return max(
            math.hypot(sx * half_l, sy * half_w + self.turning_radius)
            for sx in (1, -1) for sy in (1, -1)
        )

    def inflation_cost(self, distance: float) -> float:
        """Nav2 inflation cost at a given distance from an obstacle."""
        if distance <= self.inscribed:
            return 253.0
        return 252.0 * math.exp(-self.cost_scaling * (distance - self.inscribed))


@dataclass
class Result:
    """Outcome of one check."""

    name: str
    passed: bool
    headline: str
    rows: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


# =============================================================================
# LOADING
# =============================================================================


def load(config_path: str, dbname: str | None) -> tuple[dict[int, Node], list[Edge], list[tuple[float, float]], str]:
    """Read nodes, edges and object positions, and precompute edge geometry."""
    with open(config_path, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)

    db = config['database']
    name = dbname or db['dbname']

    conn = psycopg.connect(
        host=db['host'], port=db['port'], dbname=name,
        user=db['user'], password=db.get('password') or None,
        connect_timeout=db.get('connect_timeout', 5),
    )
    conn.autocommit = True

    try:
        with conn.cursor() as cur:
            cur.execute('SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node ORDER BY id')
            nodes = {
                int(r[0]): Node(int(r[0]), r[1], float(r[2]), float(r[3]),
                                None if r[4] is None else float(r[4]),
                                None if r[5] is None else int(r[5]))
                for r in cur.fetchall()
            }

            cur.execute('SELECT source_id, target_id, weight, edge_type FROM public.graph_edge')
            raw = cur.fetchall()

            cur.execute('SELECT x_coord, y_coord FROM public.object_data')
            objects = [(float(r[0]), float(r[1])) for r in cur.fetchall()]
    finally:
        conn.close()

    edges: list[Edge] = []
    for source, target, weight, edge_type in raw:
        source, target = int(source), int(target)
        a, b = nodes.get(source), nodes.get(target)
        if a is None or b is None:
            continue

        dx, dy = b.x - a.x, b.y - a.y
        heading = a.theta if a.theta is not None else math.atan2(dy, dx)
        edge = Edge(source, target, float(weight), edge_type)
        edge.length = math.hypot(dx, dy)
        edge.forward = dx * math.cos(heading) + dy * math.sin(heading)
        edge.lateral = -dx * math.sin(heading) + dy * math.cos(heading)
        if a.theta is not None and b.theta is not None:
            edge.dtheta = math.atan2(math.sin(b.theta - a.theta), math.cos(b.theta - a.theta))
        edges.append(edge)

    return nodes, edges, objects, name


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================


def is_sideways(edge: Edge) -> bool:
    """True when the leg is mostly across the robot rather than along it."""
    return (edge.length >= SAME_PLACE_M
            and abs(edge.lateral) > LATERAL_TOL
            and abs(edge.lateral) > abs(edge.forward))


def is_backwards(edge: Edge) -> bool:
    """True when the leg goes behind the robot, which a Dubins model forbids."""
    return edge.length >= SAME_PLACE_M and edge.forward < -LATERAL_TOL


def is_undrivable(edge: Edge) -> bool:
    """True when a forward-only planner cannot take this leg as posed."""
    return is_sideways(edge) or is_backwards(edge)


def nearest_object(node: Node, objects: list[tuple[float, float]]) -> float:
    """Distance from a node to the closest object, or inf when there are none."""
    if not objects:
        return math.inf
    return min(math.hypot(node.x - ox, node.y - oy) for ox, oy in objects)


def footprint_corners(x: float, y: float, theta: float, robot: 'Robot') -> list[tuple[float, float]]:
    """The four corners of the footprint at a pose, in map coordinates."""
    half_l, half_w = robot.length / 2.0, robot.width / 2.0
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return [
        (x + sx * half_l * cos_t - sy * half_w * sin_t,
         y + sx * half_l * sin_t + sy * half_w * cos_t)
        for sx in (1, -1) for sy in (1, -1)
    ]


def nearest_object_to_footprint(node: Node, objects: list[tuple[float, float]], robot: 'Robot') -> float:
    """
    Closest an object gets to the robot's body when it sits on this node.

    Measuring from the node centre flatters the graph: the centre can sit a
    comfortable half metre from a bush while a corner of a one-metre robot is
    almost touching it. Nav2 collision-checks the footprint, so the footprint
    is what has to be measured.
    """
    if not objects:
        return math.inf

    theta = node.theta if node.theta is not None else 0.0
    return min(
        math.hypot(cx - ox, cy - oy)
        for cx, cy in footprint_corners(node.x, node.y, theta, robot)
        for ox, oy in objects
    )


def dubins_shortest(x0, y0, t0, x1, y1, t1, radius) -> tuple[str, float] | None:
    """
    Shortest Dubins path between two poses, or None when none exists.

    Answers whether a forward-only car-like planner can join the two poses at
    all. A "no" here means no amount of Nav2 tuning will drive that edge.
    """
    def mod2pi(value: float) -> float:
        return value - 2.0 * math.pi * math.floor(value / (2.0 * math.pi))

    dx, dy = x1 - x0, y1 - y0
    distance = math.hypot(dx, dy)

    if distance < 1e-9:
        return None

    d = distance / radius
    bearing = mod2pi(math.atan2(dy, dx))
    alpha, beta = mod2pi(t0 - bearing), mod2pi(t1 - bearing)

    sa, ca, sb, cb = math.sin(alpha), math.cos(alpha), math.sin(beta), math.cos(beta)
    cab = math.cos(alpha - beta)
    options: dict[str, tuple[float, float, float]] = {}

    tmp = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if tmp >= 0:
        th = math.atan2(cb - ca, d + sa - sb)
        options['LSL'] = (mod2pi(-alpha + th), math.sqrt(tmp), mod2pi(beta - th))

    tmp = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if tmp >= 0:
        th = math.atan2(ca - cb, d - sa + sb)
        options['RSR'] = (mod2pi(alpha - th), math.sqrt(tmp), mod2pi(-beta + th))

    tmp = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if tmp >= 0:
        s = math.sqrt(tmp)
        th = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, s)
        options['LSR'] = (mod2pi(-alpha + th), s, mod2pi(-mod2pi(beta) + th))

    tmp = d * d - 2 + 2 * cab - 2 * d * (sa + sb)
    if tmp >= 0:
        s = math.sqrt(tmp)
        th = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, s)
        options['RSL'] = (mod2pi(alpha - th), s, mod2pi(beta - th))

    tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
    if abs(tmp) <= 1:
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2)
        options['RLR'] = (t, p, mod2pi(alpha - beta - t + p))

    tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8
    if abs(tmp) <= 1:
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2)
        options['LRL'] = (t, p, mod2pi(mod2pi(beta) - alpha - t + p))

    if not options:
        return None

    name = min(options, key=lambda k: sum(options[k]))
    return name, sum(options[name]) * radius


# =============================================================================
# CHECKS
# =============================================================================


def check_sideways(nodes, edges, objects, robot, limit) -> Result:
    """
    Edges a forward-only planner cannot take as posed.

    Two shapes, both fatal for Nav2 with a Dubins model: motion mostly across
    the robot, and motion behind it. Either one aborts the whole
    compute_path_through_poses request, not just that leg.
    """
    offenders = sorted((e for e in edges if is_undrivable(e)),
                       key=lambda e: -max(abs(e.lateral), -e.forward))

    by_type: dict[str, int] = defaultdict(int)
    for edge in offenders:
        by_type[edge.edge_type] += 1

    lateral_only = sum(1 for e in offenders if is_sideways(e) and not is_backwards(e))
    reversing = sum(1 for e in offenders if is_backwards(e))

    rows = [f'  {"edge":>13}  {"forward":>8} {"lateral":>8} {"length":>7}  {"why":>9}  edge_type']
    shown = offenders if limit == 0 else offenders[:limit]
    for edge in shown:
        why = 'reversing' if is_backwards(edge) else 'sideways'
        rows.append(f'  {edge.source:>5} -> {edge.target:<5}  {edge.forward:>+8.3f} {edge.lateral:>+8.3f} '
                    f'{edge.length:>7.3f}  {why:>9}  {edge.edge_type}')
    if limit and len(offenders) > limit:
        rows.append(f'  ... {len(offenders) - limit} more (--limit 0 for all)')

    notes = []
    if offenders:
        notes.append(f'  {lateral_only} sideways, {reversing} reversing')
        notes.append(f'  by edge_type: {dict(by_type)}')
        notes.append('  Each needs an approach node between the two ends; see the approach check.')

    return Result(
        'sideways',
        not offenders,
        f'{len(offenders)} of {len(edges)} edges a forward-only planner cannot take',
        rows if offenders else [],
        notes,
    )


def check_zerolen(nodes, edges, objects, robot, limit) -> Result:
    """Edges between nodes at the same place. Degenerate navigation goals."""
    offenders = [e for e in edges if e.length < SAME_PLACE_M]

    by_type: dict[str, int] = defaultdict(int)
    for edge in offenders:
        by_type[edge.edge_type] += 1

    rows = []
    shown = offenders if limit == 0 else offenders[:limit]
    for edge in shown:
        rows.append(f'  {edge.source:>5} -> {edge.target:<5}  length={edge.length:.4f} '
                    f'dtheta={edge.dtheta:+.4f} rad  {edge.edge_type}')
    if limit and len(offenders) > limit:
        rows.append(f'  ... {len(offenders) - limit} more (--limit 0 for all)')

    notes = []
    if offenders:
        notes.append(f'  by edge_type: {dict(by_type)}')
        notes.append('  Harmless if the router collapses them before publishing; fatal if it does not.')

    return Result(
        'zerolen',
        not offenders,
        f'{len(offenders)} zero-length edges',
        rows,
        notes,
    )


def propose_approach_nodes(nodes, edges) -> list[dict]:
    """
    Work out which approach nodes are missing, and where each should go.

    A row entrance is only drivable if the node the router will actually come
    from is pointing at it. Judging that on the cheapest arrival matters: an
    entrance can have one clean but expensive edge that shortest-path will
    never choose, while every affordable way in is sideways.
    """
    incoming: dict[int, list[Edge]] = defaultdict(list)
    for edge in edges:
        incoming[edge.target].append(edge)

    proposals = []

    for node in nodes.values():
        if node.node_type not in ('Entering', 'Exiting'):
            continue

        arrivals = incoming.get(node.id, [])
        if not arrivals:
            continue

        # The router takes the cheapest edge, so that is the one that decides
        # whether this entrance is reachable in practice.
        parent_edge = min(arrivals, key=lambda e: e.weight)
        if not is_undrivable(parent_edge):
            continue

        parent = nodes[parent_edge.source]

        # The approach node sits above the entrance, on the parent's line of
        # travel, facing down at it.
        approach_x, approach_y = node.x, parent.y
        heading = math.atan2(node.y - approach_y, node.x - approach_x)

        proposals.append({
            'entrance': node,
            'parent': parent,
            'x': approach_x,
            'y': approach_y,
            'theta': heading,
            'leg_in': math.hypot(approach_x - parent.x, approach_y - parent.y),
            'leg_down': math.hypot(node.x - approach_x, node.y - approach_y),
            'lateral': parent_edge.lateral,
            'forward': parent_edge.forward,
            'reason': 'sideways' if is_sideways(parent_edge) else 'reversing',
        })

    return proposals


def check_approach(nodes, edges, objects, robot, limit) -> Result:
    """Entering/Exiting nodes with no drivable way in."""
    proposals = propose_approach_nodes(nodes, edges)

    entrances = sum(1 for n in nodes.values() if n.node_type in ('Entering', 'Exiting'))

    rows = []
    if proposals:
        rows.append(f'  {"entrance":>8} {"parent":>7} {"why":>10}  {"add node at":>22} {"theta":>8}  '
                    f'{"leg in":>7} {"leg down":>8}')
        shown = proposals if limit == 0 else proposals[:limit]
        for p in shown:
            position = f'({p["x"]:.3f}, {p["y"]:.3f})'
            rows.append(f'  {p["entrance"].id:>8} {p["parent"].id:>7} {p["reason"]:>10}  {position:>22} '
                        f'{p["theta"]:>+8.4f}  {p["leg_in"]:>7.3f} {p["leg_down"]:>8.3f}')
        if limit and len(proposals) > limit:
            rows.append(f'  ... {len(proposals) - limit} more (--limit 0 for all)')

    notes = []
    if proposals:
        notes.append(f'  {len(proposals)} of {entrances} Entering/Exiting nodes have no drivable cheapest arrival.')
        notes.append('  Adding the proposed node turns each one into: drive straight, turn on the spot,')
        notes.append('  drive straight. Run with --sql to print the INSERT and edge rewiring.')

    return Result(
        'approach',
        not proposals,
        f'{len(proposals)} of {entrances} row entrances have no drivable approach',
        rows,
        notes,
    )


def check_turns(nodes, edges, objects, robot, limit) -> Result:
    """Heading changes demanded at nodes, against the robot's swept width."""
    outgoing: dict[int, list[Edge]] = defaultdict(list)
    incoming: dict[int, list[Edge]] = defaultdict(list)
    for edge in edges:
        outgoing[edge.source].append(edge)
        incoming[edge.target].append(edge)

    tight = []

    for node in nodes.values():
        if node.theta is None:
            continue

        for arrive in incoming.get(node.id, []):
            source = nodes[arrive.source]
            if source.theta is None:
                continue
            for leave in outgoing.get(node.id, []):
                turn = abs(math.atan2(math.sin(node.theta - source.theta),
                                      math.cos(node.theta - source.theta)))
                if turn < math.pi / 4:
                    continue
                clearance = nearest_object(node, objects)
                if clearance < robot.swept:
                    tight.append((node, arrive.source, leave.target, turn, clearance))

    # One row per node is enough; the same corner repeats across edge pairs.
    seen: set[int] = set()
    unique = []
    for entry in sorted(tight, key=lambda t: t[4]):
        if entry[0].id in seen:
            continue
        seen.add(entry[0].id)
        unique.append(entry)

    rows = []
    if unique:
        rows.append(f'  {"node":>6} {"type":>10} {"turn":>8} {"clearance":>10} {"swept":>7} {"margin":>8}')
        shown = unique if limit == 0 else unique[:limit]
        for node, _src, _dst, turn, clearance in shown:
            rows.append(f'  {node.id:>6} {node.node_type:>10} {math.degrees(turn):>7.1f}d '
                        f'{clearance:>10.3f} {robot.swept:>7.3f} {clearance - robot.swept:>+8.3f}')
        if limit and len(unique) > limit:
            rows.append(f'  ... {len(unique) - limit} more (--limit 0 for all)')

    notes = [
        f'  turning radius {robot.turning_radius:.3f} m, inscribed radius {robot.inscribed:.3f} m',
        f'  swept half-width while turning: {robot.swept:.3f} m (corridor {2 * robot.swept:.3f} m)',
    ]
    if robot.turning_radius < robot.inscribed:
        notes.append('  Turning radius is smaller than the robot\'s own inscribed circle, so the turn')
        notes.append('  centre lies inside the body and every arc sweeps wider, not narrower.')

    return Result(
        'turns',
        not unique,
        f'{len(unique)} nodes turn in less space than the robot sweeps',
        rows,
        notes,
    )


def check_clearance(nodes, edges, objects, robot, limit) -> Result:
    """
    Whether the robot's body can occupy each node without the costmap calling
    it a collision.

    Nav2's inflation layer paints every cell within the robot's inscribed
    radius of an obstacle with INSCRIBED_INFLATED_OBSTACLE (253), and the Smac
    collision checker refuses any pose whose footprint touches one. That band
    is fixed by the robot's own width - lowering inflation_radius shortens the
    decay tail beyond it but never shrinks the 253 band itself.

    So a node whose footprint comes within `inscribed` of an object is not a
    tuning problem. It is a pose the planner will always reject, whether it is
    the goal of a leg or a step along one.
    """
    if not objects:
        return Result('clearance', True, 'no object_data rows; nothing to measure against', [], [])

    measured = [(node, nearest_object_to_footprint(node, objects, robot)) for node in nodes.values()]

    blocked = [(n, d) for n, d in measured if d < robot.inscribed]
    tail = [(n, d) for n, d in measured if robot.inscribed <= d < robot.inflation_radius]

    rows = []
    if blocked:
        rows.append(f'  {"node":>6} {"type":>10} {"body->object":>13} {"vs inscribed":>13}')
        shown = blocked if limit == 0 else blocked[:limit]
        for node, distance in shown:
            rows.append(f'  {node.id:>6} {node.node_type:>10} {distance:>13.3f} '
                        f'{distance - robot.inscribed:>+13.3f}')
        if limit and len(blocked) > limit:
            rows.append(f'  ... {len(blocked) - limit} more (--limit 0 for all)')

    by_type: dict[str, list[float]] = defaultdict(list)
    for node, distance in measured:
        by_type[node.node_type].append(distance)

    notes = [f'  robot {robot.length:.3f} x {robot.width:.3f} m, inscribed {robot.inscribed:.3f} m',
             f'  {"node_type":<12} {"count":>6} {"min body->obj":>14} {"vs inscribed":>13} {"cost":>7}']
    for node_type in sorted(by_type):
        distances = by_type[node_type]
        closest = min(distances)
        if math.isinf(closest):
            continue
        notes.append(f'  {node_type:<12} {len(distances):>6} {closest:>14.3f} '
                     f'{closest - robot.inscribed:>+13.3f} {robot.inflation_cost(closest):>7.1f}')

    notes.append(f'  inflation_radius {robot.inflation_radius:.2f} m, cost_scaling {robot.cost_scaling:.1f}')
    notes.append(f'  {len(blocked)} nodes the planner rejects outright (footprint inside the 253 band)')
    notes.append(f'  {len(tail)} nodes in the inflation tail: passable but costed against')
    if blocked:
        notes.append('  The 253 band is the robot\'s own inscribed radius, so no inflation_radius value')
        notes.append('  clears it. Either the objects must stop entering the costmap, or the footprint')
        notes.append('  used for planning must shrink.')

    return Result(
        'clearance',
        not blocked,
        f'{len(blocked)} nodes the planner will reject, {len(tail)} in the inflation tail',
        rows,
        notes,
    )


def check_dubins(nodes, edges, objects, robot, limit) -> Result:
    """Whether a car-like planner can join each edge's two poses at all."""
    unsolvable = []
    detours = []

    for edge in edges:
        a, b = nodes[edge.source], nodes[edge.target]
        if a.theta is None or b.theta is None:
            continue
        if edge.length < SAME_PLACE_M:
            continue

        solution = dubins_shortest(a.x, a.y, a.theta, b.x, b.y, b.theta, robot.turning_radius)
        if solution is None:
            unsolvable.append(edge)
            continue

        _, length = solution
        if length > 2.0 * edge.length:
            detours.append((edge, length))

    rows = []
    if unsolvable:
        rows.append('  no Dubins solution:')
        for edge in (unsolvable if limit == 0 else unsolvable[:limit]):
            rows.append(f'    {edge.source} -> {edge.target}  {edge.edge_type}')
    if detours:
        rows.append(f'  path more than twice the straight-line distance ({len(detours)}):')
        worst = sorted(detours, key=lambda d: -d[1] / d[0].length)
        for edge, length in (worst if limit == 0 else worst[:limit]):
            rows.append(f'    {edge.source:>5} -> {edge.target:<5} straight={edge.length:>6.3f} '
                        f'dubins={length:>6.3f}  x{length / edge.length:.2f}  {edge.edge_type}')

    notes = [f'  computed at turning radius {robot.turning_radius:.3f} m',
             '  A detour factor well above 1 means the planner must loop to make the leg,',
             '  which is the signature of a missing approach node.']

    return Result(
        'dubins',
        not unsolvable,
        f'{len(unsolvable)} edges no car-like planner can join, {len(detours)} needing a detour',
        rows,
        notes,
    )


CHECKS = {
    'sideways': check_sideways,
    'zerolen': check_zerolen,
    'approach': check_approach,
    'turns': check_turns,
    'clearance': check_clearance,
    'dubins': check_dubins,
}


# =============================================================================
# SQL OUTPUT
# =============================================================================


def emit_sql(nodes, edges) -> None:
    """Print the SQL that would add the missing approach nodes."""
    proposals = propose_approach_nodes(nodes, edges)

    if not proposals:
        print('-- Nothing to add: every row entrance already has a drivable approach.')
        return

    next_id = max(nodes) + 1

    print('-- Approach nodes for row entrances that can currently only be reached sideways.')
    print('-- Review before running. Weights use the Transit multiplier of 4x euclidean.')
    print('BEGIN;')
    print()

    for offset, p in enumerate(proposals):
        node_id = next_id + offset
        entrance = p['entrance']
        parent = p['parent']

        print(f'-- entrance {entrance.id} ({entrance.node_type}), currently reached sideways '
              f'from {parent.id} (lateral {p["lateral"]:+.3f} m)')
        print(f'INSERT INTO public.graph_node (id, node_type, x, y, theta, obj_id) '
              f'VALUES ({node_id}, \'Approach\', {p["x"]:.6f}, {p["y"]:.6f}, {p["theta"]:.6f}, NULL);')
        print(f'INSERT INTO public.graph_edge (source_id, target_id, weight, edge_type) '
              f'VALUES ({parent.id}, {node_id}, {p["leg_in"] * 4.0:.6f}, \'Transit Edge\');')
        print(f'INSERT INTO public.graph_edge (source_id, target_id, weight, edge_type) '
              f'VALUES ({node_id}, {entrance.id}, {p["leg_down"] * 4.0:.6f}, \'Transit Edge\');')
        print(f'DELETE FROM public.graph_edge WHERE target_id = {entrance.id} '
              f'AND source_id <> {node_id};')
        print()

    print('COMMIT;')
    print()
    print(f'-- {len(proposals)} approach nodes, ids {next_id} .. {next_id + len(proposals) - 1}')


# =============================================================================
# ENTRY POINT
# =============================================================================


def main() -> int:
    """Run the requested checks and report a pass/fail per check."""
    parser = argparse.ArgumentParser(
        description='Grade a topological graph on whether a robot can drive it. Read-only.')
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    parser.add_argument('--dbname', default=None, help='override the database named in config.yaml')
    parser.add_argument('--checks', default='all',
                        help=f'comma-separated subset of: {",".join(ALL_CHECKS)}')
    parser.add_argument('--limit', type=int, default=10, help='rows per check, 0 for all')
    parser.add_argument('--sql', action='store_true',
                        help='print the SQL that adds the missing approach nodes, then exit')
    parser.add_argument('--robot-length', type=float, default=ROBOT_LENGTH)
    parser.add_argument('--robot-width', type=float, default=ROBOT_WIDTH)
    parser.add_argument('--turning-radius', type=float, default=TURNING_RADIUS)
    parser.add_argument('--inflation', type=float, default=INFLATION_RADIUS)
    parser.add_argument('--cost-scaling', type=float, default=COST_SCALING)

    args = parser.parse_args()

    nodes, edges, objects, dbname = load(args.config, args.dbname)

    if args.sql:
        emit_sql(nodes, edges)
        return 0

    robot = Robot(args.robot_length, args.robot_width, args.turning_radius,
                  args.inflation, args.cost_scaling)

    selected = ALL_CHECKS if args.checks == 'all' else [c.strip() for c in args.checks.split(',')]
    unknown = [c for c in selected if c not in CHECKS]
    if unknown:
        sys.exit(f'Unknown check(s): {unknown}. Choose from {ALL_CHECKS}.')

    print(f'Database : {dbname}')
    print(f'Graph    : {len(nodes)} nodes, {len(edges)} edges, {len(objects)} objects')
    print(f'Robot    : {robot.length:.3f} x {robot.width:.3f} m | inscribed {robot.inscribed:.3f} | '
          f'circumscribed {robot.circumscribed:.3f} | turning radius {robot.turning_radius:.3f}')
    print()

    failed = 0
    summary: list[tuple[str, bool, str]] = []

    for name in selected:
        result = CHECKS[name](nodes, edges, objects, robot, args.limit)
        mark = 'PASS' if result.passed else 'FAIL'
        if not result.passed:
            failed += 1

        print(f'[{mark}] {result.name}: {result.headline}')
        for row in result.rows:
            print(row)
        for note in result.notes:
            print(note)
        print()

        summary.append((result.name, result.passed, result.headline))

    print('=' * 78)
    for name, passed, headline in summary:
        print(f'  {"PASS" if passed else "FAIL"}  {name:<10} {headline}')
    print('=' * 78)
    print(f'{len(selected) - failed} of {len(selected)} checks passed.')

    if failed:
        print('Run with --sql to print the approach nodes that would fix the graph.')

    return failed


if __name__ == '__main__':
    sys.exit(main())

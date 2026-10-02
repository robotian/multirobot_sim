"""TaskGenerator — generates typed Task messages for the robot fleet."""

import math
import os
import traceback
from typing import Any

import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import shortest_path
from status_interfaces.msg import DockGoal, SubTask, Task, UndockGoal, WayPoint

from status_server.configuration import Configuration
from status_server.dataclass import DockStation, JobDetails
from status_server.enum import DockTypeEnum, NodeTypeEnum, TaskEnum
from status_server.postgres_manager import PostgresOperations

logger = RcutilsLogger(os.path.basename(__file__))

# Maximum time Nav2 is allowed to spend undocking before it gives up.
_UNDOCK_TIMEOUT = 30.0

# harvest_ready values written onto generated task rows.
_HARVEST_READY = 1
_NOT_HARVEST = 0

# Waypoints closer together than this are treated as one. Via Swap pairs sit at
# identical coordinates and differ only in heading, which is a zero-length goal
# no planner accepts.
_MIN_LEG_M = 0.05

# Heading disagreement, in radians, below which the stored node heading and the
# direction of travel are considered the same. Only used for logging which
# waypoints were re-oriented.
_HEADING_EPS = 0.05


class TaskGenerator:
    """
    Generates Task messages (with sub-tasks and waypoints) for the robot fleet.

    The node graph is built once at construction time using sparse adjacency
    matrices and scipy shortest_path, so all path queries are O(1) lookups
    against pre-computed arrays.

    Two matrices are built from the same graph because the edge weights are
    not lengths. An edge's weight is roughly its euclidean length times a
    multiplier chosen per edge type, so it ranks routes correctly but is two
    to four times larger than the distance actually driven:

        cost_matrix / predecessors - weights. Chooses the route.
        dist_matrix               - metres. Answers "how far is that?".

    Reading cost as metres would make every trip look far longer than it is,
    and the battery check in job_publisher would veto work the robot could
    comfortably finish.
    """

    def __init__(self, db_ops: PostgresOperations | None = None) -> None:
        """
        Args:
            db_ops: Database access to reuse. The task manager passes its own
                so the process holds one connection instead of two; omit it and
                a new connection is opened.
        """
        self.db_ops = db_ops if db_ops is not None else PostgresOperations()
        self.crop_type = Configuration().get_farm_config().crop_type

        # Object positions and the standoff rule are read once: the crop does
        # not move, and every published route consults them.
        self.clearance = Configuration().get_clearance_config()
        self.object_positions = self.db_ops.fetch_object_positions() if self.clearance.enabled else []

        if self.clearance.enabled:
            logger.info(
                f'Waypoint clearance on: standoff {self.clearance.standoff:.3f} m '
                f'(obstacle {self.clearance.obstacle_radius:.3f} + half-width '
                f'{self.clearance.robot_half_width:.3f} + margin {self.clearance.safety_margin:.3f}), '
                f'cap {self.clearance.max_shift:.3f} m, {len(self.object_positions)} objects.'
            )
        else:
            logger.warning('Waypoint clearance off: node positions are published unchanged.')

        cost_graph, distance_graph = self.create_adjacency_matrices()

        self.validate_dock_config()

        self.cost_matrix, self.predecessors = shortest_path(
            csgraph=cost_graph,
            method='auto',
            directed=True,
            return_predecessors=True,
        )

        self.dist_matrix = shortest_path(
            csgraph=distance_graph,
            method='auto',
            directed=True,
            return_predecessors=False,
        )

    # =========================================================================
    # TASK GENERATORS
    # =========================================================================

    def generate_harvest_task(
        self,
        robot_id: int,
        current_node_id: int,
    ) -> Task | None:
        """
        Create a harvest task:  MOVING → HARVESTING (cut and load).

        Picks the next unharvested bush side from the sweep order, writes a new
        row to farm_harvesting_job, and uses the generated row ID as task_id.

        Returns None when the whole farm has been harvested or when no path
        can be computed - never a default Task(), which would publish silently
        empty work.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
        """
        target = self._next_harvest_target()
        if target is None:
            logger.info('No unharvested bush sides remain.')
            return None

        asset_name, node_id, location = target

        # Resolve the route BEFORE writing anything. Inserting first would
        # leave an orphan open row on a path failure, and task_exists would
        # then treat that bush side as done forever.
        route = self._resolve_route(current_node_id, node_id, context='harvest')
        if route is None:
            return None

        task_id = self.db_ops.insert_task(
            node_id=node_id,
            job_schedule=asset_name,
            crop_type=self.crop_type,
            harvest_ready=_HARVEST_READY,
            assigned_robot_id=robot_id,
            location=location,
            task_type=TaskEnum.HARVESTING_TASK.value,
        )
        if task_id is None:
            return None

        return self._build_harvest_task(
            task_id=task_id,
            robot_id=robot_id,
            route=route,
            job_schedule=asset_name,
            crop_type=self.crop_type,
        )

    def generate_charging_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
    ) -> Task | None:
        """
        Create a charging task:  MOVING → DOCKING → CHARGING.

        Reserves a free charging dock from the robot's own list, writes a new
        task row, and uses the generated row ID as task_id.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
            serial_number:   Robot namespace, used to look up its docks.
        """
        return self._generate_dock_task(
            robot_id=robot_id,
            current_node_id=current_node_id,
            serial_number=serial_number,
            dock_type=DockTypeEnum.CHARGING,
            task_type=TaskEnum.CHARGING_TASK,
        )

    def generate_unloading_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
    ) -> Task | None:
        """
        Create an unloading task:  MOVING → DOCKING → UNLOADING.

        Reserves a free unloading dock, writes a new task row, and uses the
        generated row ID as task_id.

        No unloading dock has a node in the graph yet, so a robot with an
        empty `unloading` list falls back to a charging dock. With one charger
        per robot that dock is already reserved, so unloading returns None and
        the basket is never emptied - the fallback exists to keep the code
        path alive until an unloading node exists, not to work in the field.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
            serial_number:   Robot namespace, used to look up its docks.
        """
        docks = Configuration().get_robot_docks(serial_number)

        if docks.unloading:
            dock_type = DockTypeEnum.UNLOADING
        else:
            logger.warning(f'[{serial_number}] No unloading dock configured; falling back to a charging dock.')
            dock_type = DockTypeEnum.CHARGING

        return self._generate_dock_task(
            robot_id=robot_id,
            current_node_id=current_node_id,
            serial_number=serial_number,
            dock_type=dock_type,
            task_type=TaskEnum.UNLOADING_TASK,
        )

    def next_target_node(self, task_type: TaskEnum, serial_number: str) -> int | None:
        """
        Report where a task of this type would send the robot, without
        creating it.

        Lets the caller judge whether the robot can survive the trip before
        any row is written. Read-only: the dock is looked up but not reserved,
        so this deliberately ignores occupancy.

        Args:
            task_type:     Kind of task being considered.
            serial_number: Robot namespace, used to look up its docks.

        Returns:
            Target node ID, or None if no such task is available.
        """
        if task_type == TaskEnum.HARVESTING_TASK:
            target = self._next_harvest_target()
            return target[1] if target else None

        docks = Configuration().get_robot_docks(serial_number)
        dock_ids = docks.unloading if task_type == TaskEnum.UNLOADING_TASK else docks.charging

        if not dock_ids:
            dock_ids = docks.charging

        for dock_id in dock_ids:
            dock = self.db_ops.fetch_dock_by_id(dock_id)
            if dock is not None:
                return dock.node_id

        logger.warning(f'[{serial_number}] None of its docks {dock_ids} exist in dock_station.')
        return None

    def rebuild_task(
        self,
        job: JobDetails,
        current_node_id: int,
    ) -> Task | None:
        """
        Rebuild a Task message from an existing task row.

        Used to rehydrate a robot's in-flight task after a server restart. No
        row is written - the task already exists, only the message (waypoints,
        sub-tasks) has to be reconstructed from the robot's current position.

        Args:
            job:             Task row fetched from farm_harvesting_job.
            current_node_id: Node the robot currently occupies.

        Returns:
            Task, or None if the task type is unknown or no path exists.
        """
        if job.task_type == TaskEnum.HARVESTING_TASK.value:
            route = self._resolve_route(current_node_id, job.node_id, context='harvest')
            if route is None:
                return None

            return self._build_harvest_task(
                task_id=job.id,
                robot_id=job.assigned_robot_id,
                route=route,
                job_schedule=job.job_schedule,
                crop_type=job.crop_type,
            )

        if job.task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
            # job_schedule holds the dock_id for dock tasks
            dock = self.db_ops.fetch_dock_by_id(job.job_schedule)
            if dock is None:
                logger.error(f"Cannot rebuild task {job.id}: dock '{job.job_schedule}' not found.")
                return None

            task_type = TaskEnum(job.task_type)
            label = 'charging' if task_type == TaskEnum.CHARGING_TASK else 'unloading'

            route = self._resolve_route(current_node_id, dock.node_id, context=label)
            if route is None:
                return None

            task = self._build_dock_task(
                task_id=job.id,
                robot_id=job.assigned_robot_id,
                route=route,
                dock=dock,
                task_type=task_type,
            )

            # A resumed dock task must own its dock again. The reservation may
            # have been dropped while the task was paused - release_dock_for_robot
            # frees every dock the robot holds, not just the one being undocked.
            self.db_ops.reserve_dock(dock.dock_id, job.assigned_robot_id)

            return task

        logger.error(f'Cannot rebuild task {job.id}: unknown task_type {job.task_type}.')
        return None

    # =========================================================================
    # TASK BUILDERS
    # =========================================================================

    def _next_harvest_target(self) -> tuple[str, int, tuple[float, float]] | None:
        """
        Pick the next unharvested bush side, following the sweep order.

        A bush is reached from two Pickup nodes, one on each side of the row,
        and each side is a separate unit of work because the manipulator
        reaches only half the bush from either side.

        The sweep order itself comes from the database. It runs one row at a
        time, down the near lane and back up the far lane, so a row is
        finished before the next one starts:

            b1_1 .. b1_38   then   b1_38 .. b1_1   then row 2, ...

        A side is done once any task row exists for its (asset name, node)
        pair, so the farm is harvested once and then stops.

        Returns:
            (asset name, approach node ID, (x, y)) or None when nothing is left.
        """
        sweep = self.db_ops.fetch_sweep()
        if not sweep:
            logger.error('No harvestable bushes found in the database.')
            return None

        for unit in sweep:
            if not self.db_ops.task_exists(unit.name, unit.node_id):
                logger.debug(f'Next harvest target: {unit.name} via node {unit.node_id}')
                return unit.name, unit.node_id, (unit.x, unit.y)

        return None

    def _resolve_route(
        self,
        current_node_id: int,
        target_node_id: int,
        context: str = '',
    ) -> tuple[list[int], list[WayPoint]] | None:
        """
        Work out the route to a target, before anything is written.

        Task generators call this first so a pathfinding failure never leaves
        an orphan row behind: the row is only inserted once the route is known
        to exist. The resolved route is then handed to the builders, so the
        path is computed once per task rather than twice.

        Args:
            current_node_id: Node the robot currently occupies.
            target_node_id:  Node the task must reach.
            context:         Label used in the log line when no route exists.

        Returns:
            (ordered node IDs, waypoints) or None when no route exists.
        """
        node_id_list, node_list = self.get_shortest_path(current_node_id, target_node_id)

        logger.info(f'Node id list: {node_id_list}')

        if not node_id_list or node_list is None:
            logger.warning(f'No path from node {current_node_id} to {context} node {target_node_id}.')
            return None

        nodes_dict = {node.id: node for node in node_list}
        return node_id_list, self._build_waypoints(node_id_list, nodes_dict)

    def _build_harvest_task(
        self,
        task_id: int,
        robot_id: int,
        route: tuple[list[int], list[WayPoint]],
        job_schedule: str,
        crop_type: str,
    ) -> Task:
        """
        Assemble the harvest Task message from an already-resolved route.

        Writes nothing to the database and cannot fail on pathfinding, because
        the route was validated by _resolve_route before the row was inserted.
        """
        node_id_list, waypoint_list = route

        logger.debug(f'Harvest waypoints: {[w.node_id for w in waypoint_list]}')

        move_sub_task = SubTask(
            sub_task_id=1,
            type=SubTask.MOVING,
            description=f'Move to node {node_id_list[-1]}',
            data=waypoint_list,
        )

        harvest_sub_task = SubTask(
            sub_task_id=2,
            type=SubTask.HARVESTING,
            description='Harvesting the bush',
            data_str='harvest data',
        )

        task = Task(
            task_id=task_id,
            task_type=Task.HARVESTING_TASK,
            assigned_robot_id=robot_id,
            target_node_id=node_id_list[-1],
            job_schedule=job_schedule,
            description=f'Harvest Job {job_schedule}',
            crop_type=crop_type,
            sub_tasks=[move_sub_task, harvest_sub_task],
        )

        logger.debug(f'Harvest task built: {task}')
        return task

    def _generate_dock_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
        dock_type: DockTypeEnum,
        task_type: TaskEnum,
    ) -> Task | None:
        """
        Reserve a dock, persist the task row, and build the Task message.

        Shared by charging and unloading, which differ only in which dock type
        they claim and which sub-task closes the sequence.

        The dock is chosen from the robot's own configured list rather than
        from every dock of that type, so one robot cannot take another's
        charger. Robots that list the same dock share it, and the reservation
        decides who gets it.
        """
        docks = Configuration().get_robot_docks(serial_number)
        dock_ids = docks.unloading if dock_type == DockTypeEnum.UNLOADING else docks.charging

        dock = self.db_ops.fetch_free_dock_from(dock_ids, dock_type.value)
        if dock is None:
            return None

        node = self.db_ops.fetch_map_node(dock.node_id)
        if node is None:
            logger.error(f"Dock '{dock.dock_id}' points at node {dock.node_id}, which is not in the node table.")
            return None

        label = 'charging' if task_type == TaskEnum.CHARGING_TASK else 'unloading'

        # Resolve the route BEFORE writing anything, so a path failure cannot
        # leave an open row behind that nothing will ever complete.
        route = self._resolve_route(current_node_id, dock.node_id, context=label)
        if route is None:
            return None

        task_id = self.db_ops.insert_task(
            node_id=dock.node_id,
            job_schedule=dock.dock_id,
            crop_type='',
            harvest_ready=_NOT_HARVEST,
            assigned_robot_id=robot_id,
            location=(node.x, node.y),
            task_type=task_type.value,
        )
        if task_id is None:
            return None

        task = self._build_dock_task(
            task_id=task_id,
            robot_id=robot_id,
            route=route,
            dock=dock,
            task_type=task_type,
        )

        self.db_ops.reserve_dock(dock.dock_id, robot_id)

        return task

    def _build_dock_task(
        self,
        task_id: int,
        robot_id: int,
        route: tuple[list[int], list[WayPoint]],
        dock: DockStation,
        task_type: TaskEnum,
    ) -> Task:
        """
        Assemble a charging or unloading Task message from a resolved route.

        Writes nothing to the database, so it is safe to call both when
        creating a task and when rehydrating one after a restart. Cannot fail
        on pathfinding - the route was validated by _resolve_route first.
        """
        is_charging = task_type == TaskEnum.CHARGING_TASK
        label = 'charging' if is_charging else 'unloading'

        node_id_list, waypoint_list = route

        logger.debug(
            f"{label.capitalize()} dock: id='{dock.dock_id}' "
            f"type='{dock.dock_type}' "
            f'waypoints={[w.node_id for w in waypoint_list]}'
        )

        move_sub_task = SubTask(
            sub_task_id=1,
            type=SubTask.MOVING,
            description=f'Move to {label} node {node_id_list[-1]}',
            data=waypoint_list,
        )

        docking_sub_task = SubTask(
            sub_task_id=2,
            type=SubTask.DOCKING,
            description=f'Dock at {label} station',
            dock_goal=DockGoal(
                use_dock_id=True,
                dock_id=dock.dock_id,
                navigate_to_staging_pose=True,
            ),
        )

        final_sub_task = SubTask(
            sub_task_id=3,
            type=SubTask.CHARGING if is_charging else SubTask.UNLOADING,
            description=f'{"Charge" if is_charging else "Unload"} at {label} station',
            undock_goal=UndockGoal(
                dock_type=dock.dock_type,
                max_undocking_time=_UNDOCK_TIMEOUT,
            ),
        )

        task = Task(
            task_id=task_id,
            task_type=(Task.CHARGING_TASK if is_charging else Task.UNLOADING_TASK),
            assigned_robot_id=robot_id,
            target_node_id=node_id_list[-1],
            job_schedule=dock.dock_id,
            description=f'{label.capitalize()} Job -> node {node_id_list[-1]}',
            crop_type='',
            sub_tasks=[move_sub_task, docking_sub_task, final_sub_task],
        )

        logger.debug(f'{label.capitalize()} task built: {task}')
        return task

    # =========================================================================
    # PATHFINDING
    # =========================================================================

    def get_shortest_path(
        self,
        start_node_id: int,
        end_node_id: int,
    ) -> tuple[list[int] | None, list | None]:
        """
        Return (ordered node-id list, node objects) for the cheapest path.
        Both elements are None when no path exists.
        """
        route = self.shortest_path_to_destination(self.cost_matrix, self.predecessors, start_node_id, end_node_id)

        if not route:
            logger.warning(f'No path found from node {start_node_id} to node {end_node_id}')
            return None, None

        node_list = self.db_ops.fetch_route_nodes(route)

        logger.debug(f'Shortest Route: {route}')

        if not node_list:
            logger.error(f'Database returned no results for route: {route}')
            return None, None

        fetched_ids = {node.id for node in node_list}
        missing = [nid for nid in route if nid not in fetched_ids]

        logger.debug(f'Missing nodes from route: {missing}')

        if missing:
            logger.error(
                f'Route nodes missing from the node table: {missing} | '
                f'route={route} | start={start_node_id} end={end_node_id}'
            )
            return None, None

        return route, node_list

    def validate_dock_config(self) -> bool:
        """
        Check that every configured dock exists and sits on a real node.

        Switching farm.graph_source swaps which node table is live, and node
        IDs mean different places in each. A dock left pointing at the other
        schema's node fails quietly: fetch_map_node returns None, the dock task
        returns None, and the robot simply never gets sent to charge. That is
        indistinguishable from "no work available" in the logs, so it is
        checked once at start-up and reported loudly instead.

        Read-only. Problems are logged, not raised: a bad dock should not stop
        harvesting, which does not need one.

        Returns:
            True when every configured dock is usable.
        """
        source = Configuration().get_graph_source()
        docks = Configuration().get_docks()

        if not docks:
            logger.error(f"No docks configured for graph_source '{source}'. No robot can be sent to charge.")
            return False

        problems: list[str] = []
        checked: set[str] = set()

        for namespace, robot_docks in docks.items():
            dock_ids = list(robot_docks.charging) + list(robot_docks.unloading)

            if not robot_docks.charging:
                problems.append(f'{namespace}: no charging dock configured')

            for dock_id in dock_ids:
                if dock_id in checked:
                    continue
                checked.add(dock_id)

                dock = self.db_ops.fetch_dock_by_id(dock_id)
                if dock is None:
                    problems.append(f"'{dock_id}' is not in dock_station")
                    continue

                node = self.db_ops.fetch_map_node(dock.node_id)
                if node is None:
                    problems.append(
                        f"'{dock_id}' points at node {dock.node_id}, which does not exist "
                        f"under graph_source '{source}'"
                    )
                    continue

                # Existing is not enough. Regenerating the graph renumbers
                # every node, so a dock left on an old ID still resolves - to
                # whatever now happens to hold that number. Docks belong on
                # Staging nodes; anything else means the row is stale and the
                # robot will drive to the wrong place without complaint.
                node_type = getattr(node, 'node_type', None)
                if source != 'farm' and node_type != NodeTypeEnum.STAGING.value:
                    problems.append(
                        f"'{dock_id}' points at node {dock.node_id}, which is a "
                        f"'{node_type}' node, not '{NodeTypeEnum.STAGING.value}'. "
                        'The graph was probably regenerated and dock_station was not updated.'
                    )

        if problems:
            logger.error(
                f"Dock configuration is broken for graph_source '{source}':\n  " + '\n  '.join(problems)
            )
            return False

        logger.info(f"Dock configuration valid for graph_source '{source}': {sorted(checked)}")
        return True

    def create_adjacency_matrices(self) -> tuple[coo_matrix, coo_matrix]:
        """
        Build the cost and distance adjacency matrices from one edge fetch.

        Both matrices share the same sparsity pattern and differ only in what
        fills it:

            cost     - the stored edge weight, used to pick routes.
            distance - euclidean length between the edge's endpoints, in
                       metres, used to cost a trip in battery terms.

        An edge whose endpoints are missing from the node table is dropped
        from both matrices rather than silently costed as zero, which would
        make it look like a free shortcut.

        Returns:
            (cost matrix, distance matrix), both sparse and directed.

        Raises:
            RuntimeError: No edges, or no edge survived the coordinate lookup.
        """
        edge_data = self.db_ops.fetch_route_edges()
        if not edge_data:
            raise RuntimeError('Graph edge data not found in database.')

        coords = {node.id: (float(node.x), float(node.y)) for node in self.db_ops.fetch_all_map_nodes()}

        sources: list[int] = []
        targets: list[int] = []
        costs: list[float] = []
        lengths: list[float] = []
        skipped = 0

        for source, target, weight in edge_data:
            start = coords.get(source)
            end = coords.get(target)

            if start is None or end is None:
                skipped += 1
                continue

            sources.append(source)
            targets.append(target)
            costs.append(float(weight))
            lengths.append(math.dist(start, end))

        if skipped:
            logger.error(f'{skipped} edges dropped: an endpoint is missing from the node table.')

        if not sources:
            raise RuntimeError('No usable edges: every edge references a node that does not exist.')

        from_nodes = np.array(sources)
        to_nodes = np.array(targets)
        num_nodes = int(max(from_nodes.max(), to_nodes.max())) + 1
        shape = (num_nodes, num_nodes)

        cost_graph = coo_matrix((np.array(costs), (from_nodes, to_nodes)), shape=shape)
        distance_graph = coo_matrix((np.array(lengths), (from_nodes, to_nodes)), shape=shape)

        logger.info(f'Routing graph built: {num_nodes} nodes, {len(sources)} edges.')

        return cost_graph, distance_graph

    def reconstruct_path(
        self,
        predecessors: np.ndarray,
        cost_matrix: np.ndarray,
        start: int,
        end: int,
    ) -> list[int] | None:
        """
        Walk the predecessor array from end back to start and reverse.

        Guards against infinite loops caused by:
            - Corrupted predecessor cycles in the graph
            - scipy sentinel value -9999 (unreachable node marker) appearing
                despite the isinf check passing — can occur with near-zero edge
                weights that are not strictly infinite. The graph does contain
                zero-weight edges: the Via Swap hops between the clockwise and
                counter-clockwise rings cost nothing to take.
        """
        if np.isinf(cost_matrix[start, end]):
            return None

        path: list[int] = []
        visited: set[int] = set()
        i = end

        while i != start:
            # Guard 1: negative index means scipy -9999 sentinel — node is
            # unreachable despite dist_matrix not being inf (corrupted graph)
            if i < 0:
                logger.error(
                    f'reconstruct_path hit scipy sentinel (-9999) at step {len(path)} | '
                    f'start={start} end={end} | path_so_far={path}'
                )
                return None

            # Guard 2: cycle detection — same node visited twice means the
            # predecessor array contains a loop, path reconstruction would
            # never terminate
            if i in visited:
                logger.error(
                    f'reconstruct_path detected cycle at node {i} | start={start} end={end} | path_so_far={path}'
                )
                return None

            visited.add(i)
            path.append(int(i))
            i = int(predecessors[start, i])

        path.append(start)
        return path[::-1]

    def shortest_path_to_destination(
        self,
        cost_matrix: np.ndarray,
        predecessors: np.ndarray,
        start_node: int,
        end_node: int,
    ) -> list[int] | None:
        """
        Thin wrapper around reconstruct_path that swallows path errors.

        IndexError is the expected failure: the edge table sizes the cost
        matrix, so a node with no edges falls outside it.
        """
        logger.debug(f'Shortest path: {start_node} → {end_node}')
        try:
            return self.reconstruct_path(predecessors, cost_matrix, start_node, end_node)
        except (IndexError, ValueError, KeyError) as exc:
            logger.error(f'Path error {start_node}→{end_node}: {exc}\n{traceback.format_exc()}')
            return None

    # =========================================================================
    # PRIVATE HELPERS
    # =========================================================================

    def _apply_clearance(
        self,
        poses: list[tuple[int, float, float, float]],
    ) -> list[tuple[int, float, float, float]]:
        """
        Push waypoints off the crop until the robot's footprint fits.

        The graph puts a Pickup node where the manipulator wants it, about
        0.45 m from the bush centre. The crop is drawn 0.45 m thick in the map
        and the robot is 0.698 m wide, so the footprint at that node overlaps
        the mapped obstacle - a goal every planner rejects. Each waypoint is
        moved directly away from its nearest object until it stands off by
        obstacle_radius + robot_half_width + safety_margin.

        The distance gained here is lost from the manipulator's reach, one for
        one. A waypoint needing more than max_shift is moved as far as the cap
        allows and the shortfall logged, because past that point the map or the
        graph is wrong and hiding it would only move the failure later.

        Args:
            poses: (node id, x, y, theta) before the shift.

        Returns:
            The same list with positions moved. Node IDs and headings are
            untouched; headings are recomputed afterwards anyway.
        """
        if not self.clearance.enabled or not self.object_positions:
            return poses

        standoff = self.clearance.standoff
        reach = self.clearance.robot_half_length + self.clearance.obstacle_radius

        shifted: list[tuple[int, float, float, float]] = []
        moved = 0
        capped: list[tuple[int, float]] = []

        for i, (nid, x, y, theta) in enumerate(poses):
            # Push across the direction of travel, not away from the object
            # centre. Pushing away from the centre also drags the waypoint
            # along the row, which shortens the lane and moves it off the
            # bush it is meant to serve. Sideways is the only useful direction.
            heading = self._travel_heading(poses, i, theta)
            cos_h, sin_h = math.cos(heading), math.sin(heading)

            required = 0.0
            side = 1.0

            for ox, oy in self.object_positions:
                dx, dy = ox - x, oy - y

                along = dx * cos_h + dy * sin_h
                across = -dx * sin_h + dy * cos_h

                # Only objects level with the robot can touch it. The bush two
                # rows ahead is not a reason to move sideways.
                if abs(along) > reach:
                    continue

                gap = standoff - abs(across)
                if gap > required:
                    required = gap
                    # Move away from whichever side the object is on.
                    side = -1.0 if across > 0 else 1.0

            if required <= 0.0:
                shifted.append((nid, x, y, theta))
                continue

            applied = min(required, self.clearance.max_shift)

            # Unit vector across the direction of travel.
            px, py = -sin_h * side, cos_h * side
            shifted.append((nid, x + px * applied, y + py * applied, theta))
            moved += 1

            if applied < required:
                capped.append((nid, required - applied))

        if moved:
            logger.debug(f'Clearance: moved {moved} of {len(poses)} waypoints off the crop.')

        if capped:
            worst = max(capped, key=lambda c: c[1])
            logger.warning(
                f'Clearance capped at {self.clearance.max_shift:.3f} m on {len(capped)} waypoints; '
                f'node {worst[0]} is still {worst[1]:.3f} m short of standoff {standoff:.3f} m. '
                'Fix the map or the lane offset rather than raising the cap.'
            )

        return shifted

    @staticmethod
    def _travel_heading(poses: list[tuple[int, float, float, float]], index: int, fallback: float) -> float:
        """
        Direction the robot is moving through a waypoint.

        Taken from the leg leaving the waypoint, or the leg arriving at it for
        the last one. The stored heading is the fallback when neighbouring
        waypoints sit on top of each other.
        """
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

    def _drop_collinear(
        self,
        poses: list[tuple[int, float, float, float]],
    ) -> list[tuple[int, float, float, float]]:
        """
        Reduce a straight run to the waypoint at each end.

        A crop row is a line of Pickup nodes 0.44 m apart, and driving it is
        one straight leg. Publishing every node hands Nav2 twenty goals to
        reach a bush two goals away, and the robot arrives at each one before
        planning to the next. Measured on this farm, a 7.1 m row deviates from
        its own end-to-end line by 0.0007 m.

        Douglas-Peucker: keep the point furthest from the line between the two
        ends, recurse either side, and drop everything that never exceeds the
        tolerance. Corners survive because a corner is by definition far from
        that line; only genuinely straight stretches collapse.

        First and last are always kept - the last carries the heading the arm
        depends on.

        Args:
            poses: (node_id, x, y, theta) in travel order, after clearance.

        Returns:
            The same list with interior points on straight runs removed.
        """
        tolerance = self.clearance.collinear_tolerance

        if tolerance <= 0.0 or len(poses) < 3:
            return poses

        keep = [False] * len(poses)
        keep[0] = keep[-1] = True

        # Iterative Douglas-Peucker: recursion depth would otherwise track the
        # length of the route.
        stack = [(0, len(poses) - 1)]

        while stack:
            first, last = stack.pop()
            if last <= first + 1:
                continue

            x1, y1 = poses[first][1], poses[first][2]
            x2, y2 = poses[last][1], poses[last][2]
            seg_len = math.hypot(x2 - x1, y2 - y1)

            worst_i = -1
            worst_dev = 0.0

            for i in range(first + 1, last):
                px, py = poses[i][1], poses[i][2]

                if seg_len < _MIN_LEG_M:
                    # Ends coincide: no line to measure against, so fall back
                    # to distance from the shared point.
                    dev = math.hypot(px - x1, py - y1)
                else:
                    dev = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1) / seg_len

                if dev > worst_dev:
                    worst_dev = dev
                    worst_i = i

            if worst_i != -1 and worst_dev > tolerance:
                keep[worst_i] = True
                stack.append((first, worst_i))
                stack.append((worst_i, last))

        kept = [pose for pose, keep_it in zip(poses, keep) if keep_it]

        if len(kept) != len(poses):
            dropped = [pose[0] for pose, keep_it in zip(poses, keep) if not keep_it]
            logger.info(
                f'Straight-run collapse: {len(poses)} waypoints -> {len(kept)} '
                f'(dropped {dropped}, tolerance {tolerance} m)'
            )

        return kept

    def _build_waypoints(self, node_id_list: list[int], nodes_dict: dict[int, Any]) -> list[WayPoint]:
        """
        Build the WayPoint list the robot will be told to drive.

        Node headings in the graph describe how the robot should be oriented
        *at* a node - on a Pickup node the heading picks which side of the bush
        the manipulator can reach. They do not describe how to get there, and
        publishing them unchanged asks Nav2 for motion the robot cannot make.

        The ring sits 1.5 m off the row lanes, and both the ring node and the
        row's Entering node face along +x. Handing both to Nav2 as pose goals
        asks for a 1.5 m sideways step with the heading unchanged. No planner
        will produce that, and the request is aborted before the robot moves.
        Via Swap pairs are worse: two nodes at identical coordinates whose
        headings differ by pi, which is a zero-length goal.

        A third problem is positional: a Pickup node sits where the arm wants
        it, close enough to the bush that the robot's footprint overlaps the
        crop as the map draws it, which is a goal no planner accepts.

        Three passes fix all of it without touching the graph:

            1. Collapse waypoints that share a position, keeping the last of
               the run. A Via Swap is a bookkeeping hop between the clockwise
               and counter-clockwise rings, not a manoeuvre.
            2. Push each waypoint off the crop until the footprint clears it.
            3. Point every waypoint except the last at the one after it, so
               each leg is a straight drive. Turns then happen at the nodes,
               where the robot rotates in place.

        Order matters. Clearance runs before re-orienting so the headings point
        at where the robot will actually go, not where the node used to be.

        The final waypoint keeps its stored heading, because that is the one
        the arm depends on.
        """
        count = len(node_id_list)

        logger.debug(f'Node Dict: {nodes_dict}')

        # ---- Raw poses, resolving NULL theta from the travel direction ------
        # Every node in the current graph has a theta, so the NULL branch is
        # unreachable today. It is kept because a node added later without one
        # would otherwise be handed heading zero.
        poses: list[tuple[int, float, float, float]] = []

        for i, nid in enumerate(node_id_list):
            point = nodes_dict[nid]

            if point.theta is not None:
                theta = float(point.theta)
            else:
                if i > 0:
                    prev = nodes_dict[node_id_list[i - 1]]
                    dx = float(point.x) - float(prev.x)
                    dy = float(point.y) - float(prev.y)
                elif i < count - 1:
                    nxt = nodes_dict[node_id_list[i + 1]]
                    dx = float(nxt.x) - float(point.x)
                    dy = float(nxt.y) - float(point.y)
                else:
                    dx, dy = 1.0, 0.0

                theta = math.atan2(dy, dx)

                logger.debug(
                    f'Node {nid} has NULL theta — computed heading: {theta:.3f} rad (dx={dx:.3f}, dy={dy:.3f})'
                )

            poses.append((int(point.id), float(point.x), float(point.y), theta))

        # ---- Pass 1: drop zero-length legs ----------------------------------
        collapsed: list[tuple[int, float, float, float]] = []

        for pose in poses:
            if collapsed and math.dist(collapsed[-1][1:3], pose[1:3]) < _MIN_LEG_M:
                # Same place as the previous waypoint. Keep this one: it holds
                # the heading the route continues with.
                logger.debug(f'Collapsing waypoint {collapsed[-1][0]} into {pose[0]} — same position.')
                collapsed[-1] = pose
                continue
            collapsed.append(pose)

        # ---- Pass 2: stand off the crop -------------------------------------
        collapsed = self._apply_clearance(collapsed)

        # ---- Pass 3: drop waypoints on a straight run -----------------------
        # After clearance, because the shift decides where a waypoint really
        # sits, and a run only counts as straight once it has been applied.
        collapsed = self._drop_collinear(collapsed)

        # ---- Pass 4: face the next waypoint ---------------------------------
        waypoint_list: list[WayPoint] = []
        last = len(collapsed) - 1

        for i, (nid, x, y, theta) in enumerate(collapsed):
            if i < last:
                _, nx, ny, _ = collapsed[i + 1]
                heading = math.atan2(ny - y, nx - x)

                if abs(math.atan2(math.sin(heading - theta), math.cos(heading - theta))) > _HEADING_EPS:
                    logger.debug(
                        f'Node {nid}: driving heading {heading:.3f} rad replaces stored {theta:.3f} rad.'
                    )
            else:
                # Final waypoint. Its stored heading is the approach the arm
                # needs, so it is published as-is.
                heading = theta

            waypoint_list.append(WayPoint(node_id=nid, x=x, y=y, theta=heading))

        return waypoint_list


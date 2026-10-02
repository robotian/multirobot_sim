import math
import os
import traceback
from functools import partial

import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from status_interfaces.msg import SubTask, Task, WayPoint

from status_server.configuration import Configuration
from status_server.data_utils import DataUtils
from status_server.dataclass import JobPublishers, RobotEnergy, RobotLiveStatus, TopologyMapPosition
from status_server.enum import OnlineFlagEnum, ProgressStatusEnum, RobotStatusEnum, TaskEnum
from status_server.postgres_manager import PostgresOperations
from status_server.task_generator import TaskGenerator

logger = RcutilsLogger(os.path.basename(__file__))

# How often to look for robots that have come online since start-up.
_DISCOVERY_PERIOD_SEC = 5.0


class JobPublisher(Node):
    def __init__(self):
        super().__init__('job_publisher')

        self.current_task: dict[str, Task | None] = {}
        self.working_task: dict[str, Task | None] = {}
        self.previous_status: dict[str, int | None] = {}
        self.last_published_task_id: dict[str, int | None] = {}
        self.previous_online_flag: dict[str, int] = {}

        # Reverse lookup dictionary for RobotStatusEnum
        self.value_to_enum = {robotStatus.value: robotStatus for robotStatus in RobotStatusEnum}

        # Mapping Status Value to the sub-task id
        self.status_to_sub_task = {
            RobotStatusEnum.IDLE.value: 0,
            RobotStatusEnum.JOB_START.value: 0,
            # Navigation
            RobotStatusEnum.START_MOVING.value: 0,
            RobotStatusEnum.MOVING.value: 0,
            RobotStatusEnum.DESTINATION_REACHED.value: 1,
            # Harvesting
            RobotStatusEnum.START_HARVESTING.value: 1,
            RobotStatusEnum.HARVESTING.value: 1,
            RobotStatusEnum.DONE_HARVESTING.value: 2,
            # Docking
            RobotStatusEnum.START_DOCKING.value: 1,
            RobotStatusEnum.DOCKING.value: 1,
            RobotStatusEnum.DONE_DOCKING.value: 2,
            # Loading
            RobotStatusEnum.START_LOADING.value: -1,
            RobotStatusEnum.LOADING.value: -1,
            RobotStatusEnum.DONE_LOADING.value: -1,
            # Unloading
            RobotStatusEnum.START_UNLOADING.value: 2,
            RobotStatusEnum.UNLOADING.value: 2,
            RobotStatusEnum.DONE_UNLOADING.value: -1,
            # Charging
            RobotStatusEnum.START_CHARGING.value: 2,
            RobotStatusEnum.CHARGING.value: 2,
            RobotStatusEnum.DONE_CHARGING.value: -1,
            # Undocking - don't publish new tasks during undocking
            RobotStatusEnum.START_UNDOCKING.value: -1,
            RobotStatusEnum.UNDOCKING.value: -1,
            RobotStatusEnum.DONE_UNDOCKING.value: -1,
        }

        self.progress_status_map = {
            RobotStatusEnum.IDLE.value: 0,
            RobotStatusEnum.JOB_START.value: 1,
            RobotStatusEnum.PAUSED.value: 2,
            RobotStatusEnum.ERROR.value: 2,
            RobotStatusEnum.EMERGENCY_STOP.value: 2,
            RobotStatusEnum.OFFLINE.value: 2,
            RobotStatusEnum.JOB_DONE.value: 3,
        }

        # Statuses where we should NOT generate new tasks
        self.skip_new_task_statuses = [
            # RobotStatusEnum.START_UNDOCKING.value,
            # RobotStatusEnum.UNDOCKING.value,
            # RobotStatusEnum.DONE_UNDOCKING.value,
            RobotStatusEnum.START_MOVING.value,
            RobotStatusEnum.MOVING.value,
            RobotStatusEnum.DESTINATION_REACHED.value,
            RobotStatusEnum.START_DOCKING.value,
            RobotStatusEnum.DOCKING.value,
            RobotStatusEnum.START_CHARGING.value,
            RobotStatusEnum.CHARGING.value,
            RobotStatusEnum.DONE_CHARGING.value,
            RobotStatusEnum.START_HARVESTING.value,
            RobotStatusEnum.HARVESTING.value,
            RobotStatusEnum.DONE_HARVESTING.value,
            # RobotStatusEnum.START_LOADING.value,
            # RobotStatusEnum.LOADING.value,
            # RobotStatusEnum.DONE_LOADING.value,
            RobotStatusEnum.START_UNLOADING.value,
            RobotStatusEnum.UNLOADING.value,
            RobotStatusEnum.DONE_UNLOADING.value,
        ]

        self.db_ops = PostgresOperations()

        # Share this node's connection rather than letting TaskGenerator open
        # a second one - both run in the same process and the same thread.
        self.task_generator = TaskGenerator(db_ops=self.db_ops)

        self.battery_config = Configuration().get_battery_config()

        # Minimum battery level (percent, 0-100) required to accept a new task
        self.battery_reserve = float(self.battery_config.reserve_threshold)

        # Live energy picture per robot, rebuilt from the BMS feed. Held here
        # rather than in the database because this process is the one that
        # decides feasibility.
        self.robot_energy: dict[str, RobotEnergy] = {}

        self.get_logger().info(
            f'Battery reserve threshold: {self.battery_reserve:.1f}% | '
            f'safety factor: {self.battery_config.safety_factor} | '
            f'ewma alpha: {self.battery_config.ewma_alpha}'
        )

        # Node each robot returns to for charging, keyed by serial. Robots have
        # their own chargers, so a single shared value would cost every robot's
        # return trip against one other robot's dock. Resolved lazily and then
        # cached: the mapping is static, and the feasibility check runs on
        # every task assignment.
        self.charging_node_ids: dict[str, int | None] = {}

        self._init_publishers()

    def charging_node_for(self, serial_number: str) -> int | None:
        """
        Node this robot charges at, cached after the first lookup.

        Occupancy is ignored: this is used to cost the trip home, and who is
        currently parked there does not change how far away it is.

        Args:
            serial_number: Robot serial, also its namespace in the dock config.

        Returns:
            Node ID, or None when the robot has no usable charging dock.
        """
        if serial_number in self.charging_node_ids:
            return self.charging_node_ids[serial_number]

        node_id: int | None = None

        for dock_id in Configuration().get_robot_docks(serial_number).charging:
            dock = self.db_ops.fetch_dock_by_id(dock_id)
            if dock is not None:
                node_id = dock.node_id
                break

        if node_id is None:
            self.get_logger().error(
                f'[{serial_number}] No charging dock found in dock_station; battery estimates disabled.'
            )
        else:
            self.get_logger().info(f'[{serial_number}] Charging node: {node_id}')

        self.charging_node_ids[serial_number] = node_id
        return node_id

    def _init_publishers(self):
        """
        Set up per-robot publishers, and keep looking for robots that appear
        later.

        Discovery cannot be a one-off. A robot that has never published has no
        robot_live row, so starting the task manager before the robot leaves it
        with nothing to publish to - and previously that state was permanent
        until someone restarted the node. A repeating scan picks up robots as
        they come online.
        """
        self.publisher_dict: dict[str, JobPublishers] = {}

        # A dock still reserved now was left that way by a run that ended
        # without undocking. Nothing else would ever free it, and the robot
        # would be unable to charge or unload for good.
        self.db_ops.release_stale_dock_reservations()

        self.discover_robots()

        self.discovery_timer = self.create_timer(
            _DISCOVERY_PERIOD_SEC, self.discover_robots)

    def discover_robots(self) -> None:
        """
        Create publishers for robots that have come online since the last scan.

        Idempotent: robots that already have a publisher are left untouched,
        so their in-flight task and tracking state survive. Robots that go
        offline keep their publisher - the tick returns early on an offline
        status row, and tearing it down would discard the task in hand.
        """
        robot_details = self.db_ops.fetch_all_robot_details()

        if not robot_details:
            self.get_logger().error(
                'No robot details found in robot_info.', throttle_duration_sec=60.0)
            return

        self.robot_details = robot_details

        for robot_detail in robot_details:
            serial_number = robot_detail.serial_number

            # Already set up. Never rebuild: that would drop current_task.
            if serial_number in self.publisher_dict:
                continue

            robot_live_status = self.db_ops.fetch_robot_status(robot_detail.id)

            # Skip if no live status or robot is offline. robot_live.status is
            # a nullable text column, so it is coerced defensively.
            if (not robot_live_status
                    or self._as_int(robot_live_status.status,
                                    RobotStatusEnum.OFFLINE.value)
                    == RobotStatusEnum.OFFLINE.value):
                # Throttled: this repeats every scan for robots that are simply
                # not switched on.
                self.get_logger().warn(
                    f'Robot {serial_number} offline or no live status. Waiting for it to report in.',
                    throttle_duration_sec=60.0,
                )
                continue

            # publisher
            pub_topic = f'/{serial_number}/status/task'
            pub = self.create_publisher(Task, pub_topic, 10)

            # Battery telemetry. BEST_EFFORT to match the robot's own
            # publisher - a RELIABLE subscription here would receive
            # nothing at all, silently.
            self.robot_energy[serial_number] = RobotEnergy()
            self.create_subscription(
                BatteryState,
                f'/{serial_number}/platform/bms/state',
                partial(self.battery_callback, serial_number),
                qos_profile_sensor_data,
            )

            pub_timer = self.create_timer(1.0, partial(self.publish_job_callback, serial_number))

            self.publisher_dict[serial_number] = JobPublishers(
                robot_detail.id, serial_number, pub_topic, pub, pub_timer
            )

            # Initialize tracking dictionaries
            self.current_task[serial_number] = None
            self.working_task[serial_number] = None
            self.previous_status[serial_number] = None
            self.last_published_task_id[serial_number] = None
            # Seed from DB so server startup does not falsely detect a client restart
            # when the robot was already online before the server started
            self.previous_online_flag[serial_number] = self._as_int(
                robot_live_status.online_flag, OnlineFlagEnum.ONLINE.value
            )

            # No task restoration needed here: the first tick hydrates
            # current_task from the robot's open row in the database.

            self.get_logger().info(f'[{serial_number}] Created publisher for topic: {pub.topic_name}')

    # ===================== ENERGY TRACKING ======================

    def _ewma(self, previous: float | None, sample: float) -> float:
        """Blend a new sample into a running average, seeding on first use."""
        if previous is None:
            return sample
        alpha = self.battery_config.ewma_alpha
        return alpha * sample + (1.0 - alpha) * previous

    def battery_callback(self, serial_number: str, msg: BatteryState) -> None:
        """
        Fold a BMS reading into the robot's running energy picture.

        Every sample updates the power average, so idle draw, sensors and
        manipulator work are all represented - not just the cost of driving.

        BatteryState fields are NaN when unmeasured, and `current` is negative
        while discharging, so readings are validated and the sign dropped.

        Args:
            serial_number: Robot the reading belongs to.
            msg:           Battery telemetry from the robot's BMS.
        """
        energy = self.robot_energy.get(serial_number)
        if energy is None:
            return

        voltage = float(msg.voltage)
        current = float(msg.current)
        charge = float(msg.charge)

        if math.isnan(voltage):
            return

        # Remaining energy: charge (Ah) x voltage (V) = watt-hours
        if not math.isnan(charge):
            energy.energy_wh = charge * voltage

        # Draw: volts x amps = watts. Sign dropped because discharge is
        # reported negative.
        if not math.isnan(current):
            energy.avg_power_w = self._ewma(energy.avg_power_w, voltage * abs(current))

    def route_start_node(self, robot: JobPublishers, robot_live_status: RobotLiveStatus) -> int:
        """
        Decide which node a new route should start from.

        current_node_id is the nearest node to the robot's pose. That is wrong
        immediately after undocking: the staging node sits ~3 m from the ring,
        so a robot at the dock snaps onto the ring instead, and the route it
        gets starts at a node it is not standing on with no leg back out of
        the dock.

        When the robot's last task took it to a dock, that dock's node is where
        it still is, so routes start there and the drive out of the dock
        becomes the head of the next route.

        Task history alone is not enough - the robot could have been moved by
        hand. The dock node must also be the nearest or second-nearest node to
        the reported pose. Second-nearest is the rule rather than a distance,
        because the ring node is fractionally closer than the staging node the
        robot just left, and any radius that covered that would also capture
        robots legitimately working on the ring.

        Args:
            robot:             Publisher record, for the robot's database ID.
            robot_live_status: Current live status row.

        Returns:
            Node ID to start the route from.
        """
        dock_node = self.db_ops.fetch_last_dock_node(robot.id)

        if dock_node is None:
            return robot_live_status.current_node_id

        pose = self.robot_pose(robot_live_status)
        if pose is None:
            return robot_live_status.current_node_id

        # Source-agnostic: farm_node and graph_node use different ID spaces, so
        # reading the wrong schema here would rank the pose against nodes that
        # are not on the live graph at all.
        nodes = self.db_ops.fetch_navigable_map_nodes()
        if not nodes:
            return robot_live_status.current_node_id

        px, py = pose
        ranked = sorted(nodes, key=lambda n: math.hypot(float(n.x) - px, float(n.y) - py))

        if dock_node not in [n.id for n in ranked[:2]]:
            self.get_logger().debug(
                f'[{robot.serial_number}] Last task was at dock node {dock_node}, but the robot '
                f'is nearest {[n.id for n in ranked[:2]]}. Routing from {robot_live_status.current_node_id}.'
            )
            return robot_live_status.current_node_id

        if dock_node != robot_live_status.current_node_id:
            self.get_logger().info(
                f'[{robot.serial_number}] Still at dock node {dock_node}; routing from there '
                f'instead of nearest node {robot_live_status.current_node_id}.'
            )

        return dock_node

    @staticmethod
    def robot_pose(robot_live_status: RobotLiveStatus) -> tuple[float, float] | None:
        """
        Read the robot's (x, y) out of its live status row.

        Args:
            robot_live_status: Live status row holding topo_map_pos.

        Returns:
            (x, y), or None if the column is missing or unparseable.
        """
        try:
            pos = DataUtils.str_to_model(robot_live_status.topo_map_pos, TopologyMapPosition)
        except (ValueError, AttributeError, TypeError):
            return None
        return pos.x, pos.y

    def update_speed_estimate(self, serial_number: str, robot_live_status: RobotLiveStatus) -> None:
        """
        Update the robot's travel speed from successive reported positions.

        Only samples where the robot actually moved contribute. A stationary
        robot would otherwise pull the average towards zero, and the return-leg
        prediction divides by it.

        Args:
            serial_number:     Robot the position belongs to.
            robot_live_status: Current live status row, holding topo_map_pos.
        """
        energy = self.robot_energy.get(serial_number)
        if energy is None:
            return

        try:
            pos = DataUtils.str_to_model(robot_live_status.topo_map_pos, TopologyMapPosition)
        except (ValueError, AttributeError, TypeError):
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # All three are written together below, but check each: the invariant
        # is implicit, and a partial state would otherwise fault here.
        if energy.last_time is not None and energy.last_x is not None and energy.last_y is not None:
            elapsed = now - energy.last_time
            moved = math.hypot(pos.x - energy.last_x, pos.y - energy.last_y)

            if elapsed > 0.0 and moved >= self.battery_config.min_move_distance:
                energy.avg_speed_mps = self._ewma(energy.avg_speed_mps, moved / elapsed)

        energy.last_x = pos.x
        energy.last_y = pos.y
        energy.last_time = now

    def can_complete_task(self, serial_number: str, target_node_id: int, task_name: str) -> bool:
        """
        Decide whether the robot can finish a task and still reach a charger.

        Energy is predicted from measured draw and measured speed rather than
        an assumed consumption per metre:

            seconds = whole-task time + return leg distance / measured speed
            watt-hours = seconds x measured power / 3600

        The task's own navigation is already inside its configured duration,
        so only the trip back to the charger is derived from distance. That
        leg belongs to no task, which is why nothing is counted twice.

        Args:
            serial_number:  Robot under consideration.
            target_node_id: Node the task would send it to.
            task_name:      Key into battery.fixed_task_seconds.

        Returns:
            True if the task fits within the charge on board, or if there is
            not yet enough telemetry to judge.
        """
        energy = self.robot_energy.get(serial_number)
        readings = energy.readings() if energy is not None else None

        if readings is None:
            self.get_logger().debug(
                f'[{serial_number}] No battery telemetry yet; accepting {task_name} task.')
            return True

        energy_wh, avg_power_w, avg_speed_mps = readings

        charging_node_id = self.charging_node_for(serial_number)
        if charging_node_id is None:
            return True

        task_seconds = self.battery_config.fixed_task_seconds.get(task_name)
        if task_seconds is None:
            return True

        # dist_matrix is metres. cost_matrix would be two to four times larger,
        # because edge weights are traversal costs rather than lengths.
        return_m = float(self.task_generator.dist_matrix[target_node_id, charging_node_id])
        if math.isinf(return_m):
            self.get_logger().warning(
                f'[{serial_number}] No route from node {target_node_id} back to the charger.')
            return False

        predicted_s = task_seconds + return_m / avg_speed_mps
        predicted_wh = predicted_s * avg_power_w / 3600.0
        required_wh = predicted_wh * self.battery_config.safety_factor

        feasible = energy_wh > required_wh

        if not feasible:
            self.get_logger().warning(
                f'[{serial_number}] {task_name} task to node {target_node_id} not feasible: '
                f'needs {required_wh:.1f}Wh, have {energy_wh:.1f}Wh '
                f'(predicted {predicted_s / 60.0:.1f}min at {avg_power_w:.0f}W, '
                f'return {return_m:.1f}m at {avg_speed_mps:.2f}m/s)'
            )

        return feasible

    # ==================== JOB/TASK PUBLISHER ====================

    @staticmethod
    def _as_int(value, default: int) -> int:
        """
        Coerce a robot_live column to int, falling back when it is unusable.

        robot_live.status is a nullable text column and online_flag is nullable
        integer, so a row written by anything other than robot_status_sync can
        hold NULL or an empty string. Raising on that would cost the robot
        every tick with nothing but a repeating traceback to show for it.

        Args:
            value:   Raw column value.
            default: Value to use when the column is NULL or not numeric.

        Returns:
            The coerced integer, or default.
        """
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def publish_job_callback(self, serial_number: str):
        try:
            robot = self.publisher_dict.get(serial_number)

            if robot is None:
                return

            # Current robot status from database
            robot_live_status = self.db_ops.fetch_robot_status(robot.id)

            if robot_live_status is None:
                return

            # A row with NULL status is unusable for dispatch, so treat it as
            # offline rather than letting int(None) abort the tick.
            live_status = self._as_int(robot_live_status.status, RobotStatusEnum.OFFLINE.value)
            current_online_flag = self._as_int(robot_live_status.online_flag, OnlineFlagEnum.OFFLINE.value)

            # ============================================================
            # Log status transitions for debugging
            # ============================================================
            prev_status = self.previous_status.get(serial_number)
            if prev_status != live_status:
                prev_status_name = self.value_to_enum[prev_status].name if prev_status in self.value_to_enum else 'None'
                current_status_name = self.value_to_enum[live_status].name
                self.get_logger().info(
                    f'[{serial_number}] Status transition: {prev_status_name} → {current_status_name}'
                )
                self.previous_status[serial_number] = live_status

            # Travel speed is measured from reported positions, so it has to
            # be sampled on every tick, not only when a task is assigned.
            self.update_speed_estimate(serial_number, robot_live_status)

            # ============================================================
            # Hydrate from database when in-memory task is missing.
            # Covers server restart at any robot status - the task already
            # exists as a row, only the message has to be rebuilt.
            # ============================================================
            self.hydrate_task(serial_number, robot, robot_live_status)

            # ============================================================
            # Update task progress immediately on status change
            # ============================================================
            current_task = self.current_task.get(serial_number)
            if (
                prev_status != live_status
                and isinstance(current_task, Task)
                and current_task.task_type == TaskEnum.HARVESTING_TASK.value
            ):
                self.update_task_progress_to_db(live_status, serial_number)

            # ============================================================
            # CRITICAL: Skip task generation during certain statuses
            # ============================================================
            if live_status in self.skip_new_task_statuses:
                # Still publish the current task if it exists (for ongoing operations)
                if current_task and isinstance(current_task, Task):
                    sub_task_index = self.status_to_sub_task.get(live_status)
                    if (
                        sub_task_index is not None
                        and sub_task_index != -1
                        and isinstance(current_task.sub_tasks, list)
                        and sub_task_index < len(current_task.sub_tasks)
                    ):
                        sub_task = current_task.sub_tasks[sub_task_index]
                        self.publish_job(
                            robot.topic_publisher, sub_task, serial_number, float(robot_live_status.load_status)
                        )
                self.previous_online_flag[serial_number] = current_online_flag
                return

            # ============================================================
            # CLIENT RESTART HANDLING
            # ============================================================
            previous_flag = self.previous_online_flag.get(serial_number, 0)

            if previous_flag == 0 and current_online_flag == 1:
                self.client_restart_handle(
                    serial_number, robot, robot_live_status, current_task, float(robot_live_status.load_status)
                )

                # Update stored online_flag state
                self.previous_online_flag[serial_number] = current_online_flag
                return

            # ============================================================
            # Task Handler
            #
            # Tasks are only ever created at three points:
            #   ERROR + low battery - interrupt current task, go charge
            #   JOB_DONE            - close the finished task, take the next
            #   IDLE                - no task in hand, take one
            # Every other status just publishes whatever task is already held.
            # ============================================================
            if live_status == RobotStatusEnum.ERROR.value and robot_live_status.battery_level <= self.battery_reserve:
                self.handle_low_battery(serial_number, robot, robot_live_status, current_task)

            elif live_status == RobotStatusEnum.DONE_UNDOCKING.value:
                # Robot has physically left the dock - free it for others
                self.db_ops.release_dock_for_robot(robot.id)

            # Handle JOB_DONE - close the finished task and take the next one
            elif live_status == RobotStatusEnum.JOB_DONE.value:
                # Only act on entry to JOB_DONE, not on every repeated tick
                if prev_status != RobotStatusEnum.JOB_DONE.value:
                    self.complete_task(serial_number, current_task)
                    self.assign_next_task(serial_number, robot, robot_live_status)

            # Take a task when IDLE and holding none. Hydration already ran,
            # so an interrupted task would have been restored by this point.
            elif live_status == RobotStatusEnum.IDLE.value and not isinstance(current_task, Task):
                self.assign_next_task(serial_number, robot, robot_live_status)

            # ============================================================
            # Task Publishing
            # ============================================================
            current_task = self.current_task.get(serial_number)
            if current_task and isinstance(current_task, Task):
                sub_task_index = self.status_to_sub_task.get(live_status)

                # Skip if index is -1 (undocking states)
                if sub_task_index == -1 or sub_task_index is None:
                    self.previous_online_flag[serial_number] = current_online_flag
                    return

                if isinstance(current_task.sub_tasks, list) and sub_task_index < len(current_task.sub_tasks):
                    sub_task = current_task.sub_tasks[sub_task_index]

                    self.publish_job(
                        robot.topic_publisher, sub_task, serial_number, float(robot_live_status.load_status)
                    )

            # Always update online flag at end of every tick to track
            # offline → online transitions correctly
            self.previous_online_flag[serial_number] = current_online_flag

        except Exception as e:  # noqa: BLE001 - a failed tick must not kill the node
            # Deliberately broad. This runs in a timer callback, so anything
            # escaping here propagates out of rclpy.spin() and terminates the
            # process. A database blip should cost one tick, not the server.
            self.get_logger().error(f'[{serial_number}] Error: {e}\n Traceback: {traceback.format_exc()}')

    # ================== TASK LIFECYCLE HELPERS ==================

    def hydrate_task(self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus) -> None:
        """
        Rebuild the in-memory task from the database when it is missing.

        Tasks live in farm_harvesting_job, so a server restart loses only the
        Task message, never the assignment. Any row still open for this robot
        is rebuilt from the robot's current position. This replaces the old
        status-group restore logic, and works from any robot status.

        Does nothing when a task is already held, or when the robot has no
        open row.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
        """
        if isinstance(self.current_task.get(serial_number), Task):
            return

        open_job = self.db_ops.fetch_open_task(robot.id)
        if open_job is None:
            return

        task = self.task_generator.rebuild_task(
            open_job, self.route_start_node(robot, robot_live_status)
        )

        if not isinstance(task, Task):
            self.get_logger().warning(
                f'[{serial_number}] Open task {open_job.id} could not be '
                f'rebuilt. Leaving robot without a task this tick.'
            )
            return

        self.current_task[serial_number] = task
        self.last_published_task_id[serial_number] = task.task_id

        self.get_logger().info(
            f"[{serial_number}] Task restored from database: '{task.description}' (Task ID: {task.task_id})"
        )

    def complete_task(self, serial_number: str, current_task: Task | None) -> None:
        """
        Close a finished task and clear the robot's in-memory state.

        All task types are marked complete, which moves the row out of
        fetch_open_task's range so hydration will not resurrect it.

        Any dock the task held is released here as well. DONE_UNDOCKING is the
        normal release point and fires first, so this is usually a no-op - but
        a dock task that ends without ever undocking would otherwise leave the
        dock reserved forever, and nothing else would ever free it.

        Args:
            serial_number: Robot serial, key into the tracking dicts.
            current_task:  Task the robot just finished, if one is held.
        """
        if isinstance(current_task, Task):
            self.get_logger().info(
                f'[{serial_number}] Job Done: Task ID: {current_task.task_id} | '
                f'Target Node: {current_task.target_node_id} | '
                f'Description: {current_task.description}'
            )

            try:
                self.db_ops.mark_job_complete(current_task.task_id, current_task.job_schedule)
            except Exception as e:  # noqa: BLE001 - never let a DB fault stall the tick
                self.get_logger().error(f'[{serial_number}] Failed to mark job complete: {e}')

            if current_task.task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
                robot = self.publisher_dict.get(serial_number)
                if robot is not None:
                    self.db_ops.release_dock_for_robot(robot.id)

        self.current_task[serial_number] = None
        self.last_published_task_id[serial_number] = None
        self.working_task[serial_number] = None

    def assign_next_task(self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus) -> None:
        """
        Create and assign the robot's next task.

        Falls back to a charging task when no work remains, so the robot parks
        at the charging station rather than sitting in the field.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
        """
        new_task = self.fetch_task_for_robot(robot_live_status, robot)

        if new_task is None:
            new_task = self.task_generator.generate_charging_task(
                robot.id, self.route_start_node(robot, robot_live_status), serial_number
            )

            if isinstance(new_task, Task):
                self.get_logger().info(f'[{serial_number}] No work remaining. Sending robot to charging station.')

        if not isinstance(new_task, Task):
            self.get_logger().warning(f'[{serial_number}] No task could be assigned this tick.')
            return

        self.current_task[serial_number] = new_task
        self.last_published_task_id[serial_number] = new_task.task_id

        self.get_logger().info(
            f'[{serial_number}] New Task: {new_task.description} | '
            f'Task ID: {new_task.task_id} | '
            f'Battery: {robot_live_status.battery_level:.2f}% | '
            f'Load: {robot_live_status.load_status:.2f}%'
        )

    def handle_low_battery(
        self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus, current_task: Task | None
    ) -> None:
        """
        Interrupt the current task and send the robot to charge.

        Skips when the robot is already on a charging task, which keeps this
        idempotent across repeated ERROR ticks without needing to track the
        status transition.

        The interrupted task is left PAUSED rather than complete, so it is
        picked back up by hydration once the robot has charged.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
            current_task:      Task being interrupted, if one is held.
        """
        if isinstance(current_task, Task) and current_task.task_type == TaskEnum.CHARGING_TASK.value:
            return

        if isinstance(current_task, Task):
            self.db_ops.update_task_progress(
                current_task.task_id, current_task.job_schedule, ProgressStatusEnum.PAUSED.value
            )

            # Give up any dock the interrupted task was holding, otherwise the
            # robot would hold two reservations at once and the unloading dock
            # would stay claimed for the whole charging trip. The reservation is
            # taken again by rebuild_task when the paused task resumes.
            if current_task.task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
                self.db_ops.release_dock_for_robot(robot.id)

            self.get_logger().warning(
                f'[{serial_number}] Battery {robot_live_status.battery_level:.2f}% '
                f'at or below reserve {self.battery_reserve:.1f}%. '
                f'Pausing task {current_task.task_id} '
                f"'{current_task.description}'."
            )

        charging_task = self.task_generator.generate_charging_task(
            robot.id, self.route_start_node(robot, robot_live_status), serial_number
        )

        if not isinstance(charging_task, Task):
            self.get_logger().error(f'[{serial_number}] Low battery but no charging task could be created.')
            return

        self.current_task[serial_number] = charging_task
        self.last_published_task_id[serial_number] = charging_task.task_id

        self.get_logger().info(
            f'[{serial_number}] New Charging Task: {charging_task.description} | '
            f'Task ID: {charging_task.task_id} | '
            f'Battery: {robot_live_status.battery_level:.2f}%'
        )

    def client_restart_handle(self, serial_number, robot, robot_live_status, current_task, crop_load: float):
        """Handles the event when a client robot node restarts and comes back online."""
        self.get_logger().info(f'[{serial_number}] Client robot node has restarted. Re-publishing current task...')

        if (
            isinstance(current_task, Task)
            and isinstance(current_task.sub_tasks, list)
            and len(current_task.sub_tasks) > 0
        ):
            # Re-publish first sub-task to sync client
            sub_task = current_task.sub_tasks[0]
            self.publish_job(robot.topic_publisher, sub_task, serial_number, crop_load)
            self.get_logger().info(
                f"[{serial_number}] Re-published task '{current_task.description}' "
                f'(Task ID: {current_task.task_id}) after client restart.'
            )
        else:
            # Robot restarted but no current task known — fetch new task
            self.get_logger().warning(
                f'[{serial_number}] Robot restarted but no active task found. Fetching appropriate task.'
            )
            new_task = self.fetch_task_for_robot(robot_live_status, robot)

            if isinstance(new_task, Task) and isinstance(new_task.sub_tasks, list):
                self.current_task[serial_number] = new_task
                self.last_published_task_id[serial_number] = new_task.task_id
                sub_task = new_task.sub_tasks[0]
                self.publish_job(robot.topic_publisher, sub_task, serial_number, crop_load)
                self.get_logger().info(f"[{serial_number}] Assigned new task '{new_task.description}' after restart.")

    def publish_job(self, publisher: Publisher, sub_task: SubTask, serial_number: str, crop_load: float):
        """
        Publish a job to a specified publisher.
        """

        self.get_logger().debug(f'Current Crop load: {crop_load:.2f} %')
        current_task = self.current_task.get(serial_number)

        if not isinstance(current_task, Task):
            return

        st = SubTask(
            sub_task_id=sub_task.sub_task_id,
            type=sub_task.type,
            description=sub_task.description,
            data=sub_task.data,
            data_str=sub_task.data_str,
            dock_goal=sub_task.dock_goal,
            undock_goal=sub_task.undock_goal,
        )

        task_msg = Task(
            task_id=current_task.task_id,
            task_type=current_task.task_type,
            assigned_robot_id=current_task.assigned_robot_id,
            target_node_id=current_task.target_node_id,
            job_schedule=current_task.job_schedule,
            description=current_task.description,
            crop_type=current_task.crop_type,
            crop_load=crop_load,
            sub_tasks=[st],
        )

        self.working_task[serial_number] = task_msg
        publisher.publish(task_msg)

    def fetch_task_for_robot(self, robot_status: RobotLiveStatus, robot_detail: JobPublishers) -> Task | None:
        """
        Decide and create the appropriate next task for a robot.

        The battery check runs BEFORE any task is generated. Generators now
        write a row to the database, so generating a task and then discarding
        it on a failed battery check would leave an orphaned open row that
        hydration would later resurrect.

        Order of preference:
            battery at or below reserve -> charging task
            crop load full              -> unloading task
            otherwise                   -> harvest task

        Both gates run before generation. Generators write a row, so a task
        rejected afterwards would leave an orphan that hydration resurrects.

        Returns:
            Task: Generated task
            None: No tasks available
        """
        serial = robot_detail.serial_number
        start_node = self.route_start_node(robot_detail, robot_status)
        self.get_logger().debug(f'[{serial}] Current load: {robot_status.load_status}')

        # Reported percentage gate - nothing is worth starting below reserve
        if robot_status.battery_level <= self.battery_reserve:
            self.get_logger().info(
                f'[{serial}] Battery '
                f'{robot_status.battery_level:.2f}% at or below reserve '
                f'{self.battery_reserve:.1f}%. Charging before new work.'
            )
            return self.task_generator.generate_charging_task(robot_detail.id, start_node, serial)

        if robot_status.load_status >= 100:
            task_type, task_name = TaskEnum.UNLOADING_TASK, 'unloading'
        else:
            task_type, task_name = TaskEnum.HARVESTING_TASK, 'harvesting'

        # Measured-energy gate. Asks where the task would go before creating
        # it, so an unaffordable task is never written.
        target_node_id = self.task_generator.next_target_node(task_type, serial)
        if target_node_id is None:
            return None

        if not self.can_complete_task(serial, target_node_id, task_name):
            self.get_logger().info(f'[{serial}] Charging instead of the {task_name} task it cannot finish.')
            return self.task_generator.generate_charging_task(robot_detail.id, start_node, serial)

        if task_type == TaskEnum.UNLOADING_TASK:
            return self.task_generator.generate_unloading_task(robot_detail.id, start_node, serial)

        return self.task_generator.generate_harvest_task(robot_detail.id, start_node)

    # ================= DATABASE UPDATE METHODS ==================

    def update_task_progress_to_db(self, robot_status: int, serial_number: str):
        """Update task progress in database."""
        progress_status = self.progress_status_map.get(robot_status)
        current_task = self.current_task.get(serial_number)

        if progress_status is not None and isinstance(current_task, Task):
            self.db_ops.update_task_progress(current_task.task_id, current_task.job_schedule, progress_status)

    # ====================== HELPER METHODS ======================

    def calculate_total_distance(self, waypoints: list[WayPoint]) -> float:
        """Calculate total distance between waypoints."""
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]

            distance = math.sqrt((wp2.x - wp1.x) ** 2 + (wp2.y - wp1.y) ** 2)
            total_distance += distance
        return total_distance


    # ========================= NODE LIFECYCLE =========================

    def destroy_node(self):
        """
        Close the database connection before destroying the node.

        One close suffices: TaskGenerator shares this node's connection rather
        than holding its own.
        """
        self.db_ops.close()
        super().destroy_node()


def main(args=None) -> None:
    """
    Entry point for the Task Manager node.

    Construction happens inside the try block so a failure in __init__ - a bad
    config or an unreachable database - still reaches the cleanup path instead
    of aborting the process with an open connection.

    Teardown lives in finally because rclpy.spin() raises on Ctrl-C: anything
    placed after it would never run, which previously leaked the database
    connections on every interrupt.
    """
    node = None
    try:
        rclpy.init(args=args)
        node = JobPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Task Manager interrupted by user. JobPublisher Node has been shutdown.')
    except Exception as e:  # noqa: BLE001 - top-level guard so shutdown still runs
        logger.error(f'Runtime error: {e}\n Traceback: {traceback.format_exc()}')
    finally:
        if node is not None:
            node.destroy_node()
        # Guarded: rclpy.shutdown() raises if init() never succeeded, which
        # would bury whatever real error brought us here.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

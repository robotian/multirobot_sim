"""
Robot Status Monitor Node.

This module defines the `RobotSubscriber` class, a ROS2 node that subscribes to robot status updates
and maintains database records. The node listens to messages on the robot status topic and updates
the database with real-time robot status information.

Classes:
    RobotSubscriber:
        A ROS2 node that subscribes to robot status updates and manages database records.

Functions:
    main(args=None):
        Initializes and runs the `RobotSubscriber` node.

Dependencies:
    - rclpy: ROS2 Python client library for creating nodes and handling communication.
    - status_interfaces.msg.RobotStatus: Custom message type for robot status updates.
    - status_server.dataclass: Contains data structures.
    - status_server.postgres_operations: Handles database operations.

Usage:
    Run this script as a standalone ROS2 node to subscribe to robot status updates and
    maintain the database records for real-time robot status.

    Ensure that the database is properly configured with the required tables (e.g., 'robot_live')
    and that robot details are pre-populated in the database before running this node.

Exceptions:
    - Logs errors if database operations fail or required tables do not exist.
    - Logs errors if invalid message types are received or if robot details are missing.
"""
import math
import traceback
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from psycopg import Error as PsycopgError
from psycopg import sql
from rclpy.node import Node
from status_interfaces.msg import RobotStatus
from tf_transformations import euler_from_quaternion

from status_server.configuration import Configuration
from status_server.dataclass import NamespaceConfig, Point, RobotDetails, RobotLive, Time, TopologyMapPosition
from status_server.postgres_manager import PostgresOperations


class RobotStatusSync(Node):
    """
    A ROS2 node that subscribes to robot status updates and maintains database records.

    This class implements a ROS2 node that listens to robot status messages and manages
    the corresponding database entries. It handles the real-time status updates of robots
    by receiving messages on the robot status topic and updating a database accordingly.

    Attributes:
        db_ops (PostgresOperations): Database operations instance for handling robot status.
        config (Configuration): Configuration instance for namespace and settings.

    Subscribes to:
        /{robot_namespace}/status/robot (RobotStatus): Topic for receiving robot status updates.

    Database Operations:
        - Fetches robot details based on robot namespace
        - Updates or inserts robot live status in the robot_live table
        - Maintains real-time robot status information in the database

    Note:
        The node expects the database to have a 'robot_live' table and appropriate robot
        details stored before it can successfully process status updates.
    """

    def __init__(self):
        """
        Initialize the RobotSubscriber node.

        This constructor sets up the RobotSubscriber node by performing the following:
        - Calls the parent class initializer with the node name 'robot_subscriber'.
        - Initializes database operations using PostgresOperations.
        - Creates subscriptions for each robot's status topic based on configuration.
        - Logs the topics to which the node subscribes.

        Attributes:
            db_ops (PostgresOperations): An instance for handling database operations.
            config (Configuration): Configuration instance.
            pose (PoseWithCovarianceStamped): Pose message placeholder.
        """
        super().__init__('robot_subscriber')

        self.config = Configuration()
        self.db_ops = PostgresOperations()
        self.pose = PoseWithCovarianceStamped()

        namespace_config: NamespaceConfig = self.config.get_namespace_config()

        if namespace_config.mode == 'single':
            self.namespace = namespace_config.namespace
            watched = [namespace_config.namespace]

            self.robot_sub = self.create_subscription(
                RobotStatus,
                f"{self.namespace}/status/robot",
                self.listener_callback,
                10
            )

            self.get_logger().info(
                f"Subscribed to topic: {self.robot_sub.topic_name}")

        else:
            watched = list(namespace_config.namespaces or [])

            if namespace_config.namespaces:
                for ns in namespace_config.namespaces:
                    sub = self.create_subscription(
                        RobotStatus,
                        f'{ns}/status/robot',
                        self.listener_callback,
                        10
                    )

                    self.get_logger().info(
                        f"Subscribed to topic: {sub.topic_name}")

        self._init_liveness_watchdog(watched)

    # =========================================================================
    # LIVENESS WATCHDOG
    # =========================================================================

    def _init_liveness_watchdog(self, watched_namespaces: list[str]) -> None:
        """
        Start the watchdog that marks silent robots offline.

        A robot that crashes or drops off the network simply stops publishing,
        so nothing ever writes OFFLINE and its robot_live row freezes at the
        last reported values. The task manager would go on handing work to it.
        Message arrival time is the only liveness signal available.

        Serials are seeded from the namespace config rather than discovered on
        first message, so a robot that never appears at all is still reported
        offline once the timeout passes.

        Args:
            watched_namespaces: Namespaces from config, with or without a
                leading slash.
        """
        liveness = self.config.get_liveness_config()
        self.liveness_timeout = float(liveness.timeout)

        now = self._now_seconds()

        # RobotStatus.robot_namespace arrives with slashes stripped, so the
        # config form ('/a300_00036') is normalised to match ('a300_00036').
        self.last_seen: dict[str, float] = {
            ns.replace('/', ''): now for ns in watched_namespaces if ns
        }

        # Serials already written as offline, so the watchdog issues one UPDATE
        # per outage instead of one per tick.
        self.offline_marked: set[str] = set()

        # Serials already warned about. Kept separate from offline_marked
        # because a robot with no robot_live row is retried every tick until
        # the row appears, and that must not log on every retry.
        self.silent_logged: set[str] = set()

        self.liveness_timer = self.create_timer(
            float(liveness.check_period), self.liveness_check)

        self.get_logger().info(
            f"Liveness watchdog started: timeout={self.liveness_timeout:.1f}s "
            f"check_period={liveness.check_period:.1f}s "
            f"watching={sorted(self.last_seen)}")

    def _now_seconds(self) -> float:
        """Current node time in seconds, following use_sim_time."""
        return self.get_clock().now().nanoseconds / 1e9

    def _record_seen(self, serial_number: str) -> None:
        """
        Note that a robot just reported in.

        Robots not listed in the namespace config are tracked too, so anything
        publishing on a subscribed topic is covered.

        Args:
            serial_number: RobotStatus.robot_namespace, slashes already stripped
                by the publishing robot.
        """
        self.last_seen[serial_number] = self._now_seconds()

        if serial_number in self.offline_marked:
            self.offline_marked.discard(serial_number)
            self.get_logger().info(
                f"[{serial_number}] Back online after being marked offline.")

        self.silent_logged.discard(serial_number)

    def liveness_check(self) -> None:
        """
        Mark robots offline once they have been silent past the timeout.

        Warning and write are tracked separately. A robot whose robot_live row
        does not exist yet cannot be marked, so the write is retried on every
        tick until the row appears - but the warning is emitted only once per
        outage, otherwise those retries would fill the log.

        Recovery needs no reset here: the robot's next status message
        overwrites both status and online_flag through the normal update path.
        """
        now = self._now_seconds()

        # Snapshot: _record_seen can add keys, and the executor may become
        # multi-threaded later, either of which would break live iteration.
        for serial, last_seen in list(self.last_seen.items()):
            silent_for = now - last_seen

            if silent_for <= self.liveness_timeout:
                continue

            if serial not in self.silent_logged:
                self.get_logger().warning(
                    f"[{serial}] No status message for {silent_for:.1f}s "
                    f"(timeout {self.liveness_timeout:.1f}s). Marking offline.")
                self.silent_logged.add(serial)

            if serial in self.offline_marked:
                continue

            # False also covers "no robot_live row yet", so this retries until
            # there is something to write to.
            if self.db_ops.mark_robot_offline(serial):
                self.offline_marked.add(serial)

    def listener_callback(self, msg: RobotStatus):
        """
        Handle incoming RobotStatus messages.

        This function processes incoming messages of type RobotStatus, retrieves
        corresponding robot details from the database, and updates the robot's
        live status in the database.

        UNIT CONTRACT: msg.battery_level arrives as a percentage in the
        0 - 100 range, already normalised by the publishing robot. It is
        written to robot_live unchanged.

        Args:
            msg (RobotStatus): The incoming message containing the robot's status.

        Raises:
            TypeError: If the incoming message is not of type RobotStatus.

        Database Operations:
            - Fetches robot details from the database using the robot's namespace.
            - Converts the message timestamp to a datetime object.
            - Creates a RobotLive object with the current status.
            - Checks if the 'robot_live' table exists in the database.
            - Updates the robot's live status if the robot already exists in the table.
            - Inserts a new robot live status if the robot does not exist in the table.
        """
        try:
            if not isinstance(msg, RobotStatus):
                self.get_logger().error(
                    "Error: Invalid message type received. "
                    f"Expected RobotStatus, received {type(msg)}")
                return

            robot_status: RobotStatus = msg

            # Liveness: record arrival before anything that can fail or return
            # early, so a robot that is publishing is never called dead just
            # because its database row is missing.
            self._record_seen(robot_status.robot_namespace)

            # Fetch robot details from database
            robot_details_from_db = self.db_ops.fetch_robot_details(
                robot_status.robot_namespace)

            if not robot_details_from_db:
                self.get_logger().error(
                    "Error: No robot details found for namespace "
                    f"{robot_status.robot_namespace}.")
                return

            robot_details = [RobotDetails(*row)
                             for row in robot_details_from_db]

            if len(robot_details) != 1:
                self.get_logger().error(
                    f"Error: Robot with namespace {robot_status.robot_namespace} not found "
                    "in database or multiple instances found.")
                return

            robot_detail: RobotDetails = robot_details[0]

            msg_time = self.get_clock().now().from_msg(msg.header.stamp)

            # Convert quaternion to Euler angles
            q = robot_status.topo_map_orientation
            quaternion = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Find closest node. Yaw is passed so the two one-way lanes of a
            # row can be told apart; position alone cannot separate them.
            closest_node = self.get_closest_node(
                float(robot_status.topo_map_position.x),
                float(robot_status.topo_map_position.y),
                float(yaw))

            # Create RobotLive object
            current_robot_status = RobotLive(
                id=robot_detail.id,
                time=str(datetime.fromtimestamp(
                    msg_time.nanoseconds/10**9, tz=timezone.utc)),
                ros_time=Time(sec=msg.header.stamp.sec,
                              nanosec=msg.header.stamp.nanosec),
                robot_type=robot_detail.robot_type,
                online_flag=robot_status.online_flag,
                status=robot_status.status,
                task=robot_status.task,
                current_node_id=closest_node,
                target_node_id=robot_status.target_node_id,
                # RobotStatus.battery_level is already normalised to 0 - 100 by
                # the publishing robot (see husky_operations_manager
                # _set_battery_status). Stored as-is; do not rescale here.
                battery_level=robot_status.battery_level,
                operation_hours_after_charging=robot_status.operation_hours_after_charging,
                load_status=robot_status.load_status,
                crop_type=robot_status.crop_type,
                gps_location=Point(
                    lat=robot_status.gps_location.x,
                    lon=robot_status.gps_location.y
                ),
                topo_map_pos=TopologyMapPosition(
                    x=robot_status.topo_map_position.x,
                    y=robot_status.topo_map_position.y,
                    z=robot_status.topo_map_position.z,
                    r=roll,
                    p=pitch,
                    w=yaw
                )
            )

            self.get_logger().debug(f"[{robot_detail.serial_number}] Crop load: {robot_status.load_status} %")

            # Check if table exists and update/insert accordingly
            if self.db_ops.db.table_exists('robot_live'):
                if self.db_ops.db.row_exists('robot_live', current_robot_status.id):
                    self.update_robot_live_status(current_robot_status)
                else:
                    self.insert_robot_live_status(current_robot_status)
            else:
                self.get_logger().error(
                    'Error: Table robot_live does not exist in database.')

        except Exception as e:  # noqa: BLE001 - one bad message must not kill the subscription
            self.get_logger().error(
                f"Error in listener_callback: {e}\n Traceback: {traceback.format_exc()}")

    def insert_robot_live_status(self, data: RobotLive):
        """
        Insert a new record into the robot_live table with the provided robot live status data.

        Args:
            data (RobotLive): An instance of RobotLive containing the robot's live status info.

        Raises:
            Exception: If there is an error during the database operation.
        """
        try:
            query = sql.SQL("""
                INSERT INTO robot_live (
                    id, time, ros_time, robot_type, online_flag, status,
                    gps_location, task, topo_map_pos, current_node_id,
                    target_node_id, battery_level, operation_hours_after_charging,
                    load_status, crop_type
                ) VALUES (
                    %s, %s, %s, %s, %s, %s,
                    Point(%s, %s), %s, (%s, %s, %s, %s, %s, %s), %s,
                    %s, %s, %s, %s, %s
                )
            """)

            params = (
                data.id,
                data.time,
                str(data.ros_time),
                data.robot_type,
                data.online_flag,
                data.status,
                data.gps_location.lat,
                data.gps_location.lon,
                data.task,
                data.topo_map_pos.x,
                data.topo_map_pos.y,
                data.topo_map_pos.z,
                data.topo_map_pos.r,
                data.topo_map_pos.p,
                data.topo_map_pos.w,
                data.current_node_id,
                data.target_node_id,
                data.battery_level,
                data.operation_hours_after_charging,
                data.load_status,
                data.crop_type
            )

            success = self.db_ops.db.execute_query(query, params)
            if not success:
                self.get_logger().error("Failed to insert robot live status")

        except (PsycopgError, TypeError, ValueError, AttributeError) as e:
            self.get_logger().error(
                f"Error inserting robot live status: {e}\n {traceback.format_exc()}")

    def update_robot_live_status(self, data: RobotLive):
        """
        Update the live status of a robot in the database.

        Args:
            data (RobotLive): An instance of RobotLive containing the robot's status information.

        Raises:
            Exception: If there is an error during the database update operation.
        """
        try:
            query = sql.SQL("""
                UPDATE robot_live SET
                    time = %s,
                    robot_type = %s,
                    ros_time = %s,
                    online_flag = %s,
                    status = %s,
                    gps_location = Point(%s, %s),
                    task = %s,
                    topo_map_pos = (%s, %s, %s, %s, %s, %s),
                    current_node_id = %s,
                    target_node_id = %s,
                    battery_level = %s,
                    operation_hours_after_charging = %s,
                    load_status = %s,
                    crop_type = %s
                WHERE id = %s
            """)

            params = (
                data.time,
                data.robot_type,
                str(data.ros_time),
                data.online_flag,
                data.status,
                data.gps_location.lat,
                data.gps_location.lon,
                data.task,
                data.topo_map_pos.x,
                data.topo_map_pos.y,
                data.topo_map_pos.z,
                data.topo_map_pos.r,
                data.topo_map_pos.p,
                data.topo_map_pos.w,
                data.current_node_id,
                data.target_node_id,
                data.battery_level,
                data.operation_hours_after_charging,
                data.load_status,
                data.crop_type,
                data.id
            )

            success = self.db_ops.db.execute_query(query, params)
            if not success:
                self.get_logger().error("Failed to update robot live status")

        except (PsycopgError, TypeError, ValueError, AttributeError) as e:
            self.get_logger().error(
                f"Error updating robot live status: {e}\n {traceback.format_exc()}")

    def get_closest_node(self, x: float, y: float, yaw: float) -> int:
        """
        Find the node the robot is standing on, matching heading as well as
        position.

        Position alone is not enough. A row is driven as two one-way lanes,
        one on each side of the bushes, and those lanes are only about 0.9 m
        apart. Half a metre of localisation error is therefore enough to snap
        the robot onto the lane running the other way. Nothing fails visibly
        when that happens - the graph is strongly connected, so a route is
        still found - but it sends the robot out of the row, around the ring
        and back in the far end, and the first waypoint asks it to turn around
        inside a lane too narrow to turn around in.

        Heading disambiguates the two lanes cleanly: their nodes face opposite
        directions, so only one of them can be within 90 degrees of the way
        the robot is actually pointing.

        Only nodes with at least one edge are considered. An edgeless node
        sits outside the routing matrix, so reporting one as the robot's
        position makes every later path lookup fail and the robot silently
        stops receiving tasks.

        Args:
            x (float): The x-coordinate of the robot in the map frame.
            y (float): The y-coordinate of the robot in the map frame.
            yaw (float): The robot's heading in radians.

        Returns:
            int: The ID of the closest node the robot could be facing along.

        Raises:
            ValueError: If no navigable nodes are found in the database.
            Exception: If there is an issue connecting to the database or
                       executing the query.
        """
        try:
            node_list = self.db_ops.fetch_navigable_map_nodes()

            if not node_list:
                raise ValueError("No navigable nodes found in the database.")

            def _distance(node) -> float:
                return math.hypot(node.x - x, node.y - y)

            def _facing_same_way(node) -> bool:
                if node.theta is None:
                    return True
                difference = math.atan2(
                    math.sin(float(node.theta) - yaw),
                    math.cos(float(node.theta) - yaw),
                )
                return abs(difference) < math.pi / 2

            aligned = [node for node in node_list if _facing_same_way(node)]

            if not aligned:
                # Robot is broadside to every node, which happens while it is
                # turning at a junction. Fall back to position alone rather
                # than refuse to report a position, but say so: if this is not
                # rare, the map headings and the robot's frame disagree.
                self.get_logger().warning(
                    f"No node within 90 deg of yaw {yaw:.2f} rad at ({x:.2f}, {y:.2f}); "
                    "snapping on position alone."
                )
                aligned = node_list

            return min(aligned, key=_distance).id

        except Exception as e:
            self.get_logger().error(f"Error finding closest node: {e}")
            raise

    def destroy_node(self):
        """
        Clean up resources before destroying the node.
        """
        self.db_ops.close()
        super().destroy_node()


def main(args=None) -> None:
    """
    Entry point for the Robot Subscriber node.

    This function initializes the ROS 2 client library, creates an instance
    of the RobotStatusSync node, and starts spinning to process incoming
    messages. It also handles proper shutdown of the node and the ROS 2
    client library. Exceptions are logged appropriately.

    Args:
        args (list, optional): Command-line arguments passed to the ROS 2
            client library. Defaults to None.

    Raises:
        Exception: Logs the stack trace for any exception other than
            KeyboardInterrupt.
    """
    subscriber = None
    try:
        rclpy.init(args=args)
        subscriber = RobotStatusSync()
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        if subscriber:
            subscriber.get_logger().info('Node Stopped.')
    except Exception as e:  # noqa: BLE001 - top-level guard so shutdown still runs
        if subscriber:
            subscriber.get_logger().error(
                f"Runtime error: {e}\n Traceback: {traceback.format_exc()}")
    finally:
        if subscriber:
            subscriber.destroy_node()
        # Guarded: on Ctrl-C rclpy's signal handler has already shut the context
        # down, and a second rclpy.shutdown() raises (as it does if init() failed).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

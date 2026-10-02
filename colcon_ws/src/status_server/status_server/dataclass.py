"""
This module defines various data classes used in the Robot Status Monitor system.
Classes:
    Point: Represents a geographical point with latitude and longitude.
    TopologyMapPosition: Represents a node in a topological map with coordinates and orientation.
    Time: Represents a time with seconds and nanoseconds.
    RobotJobPublishers: Represents publishers and subscribers for robot jobs.
    DatabaseDetails: Represents details required to connect to a database.
    Config: Represents configuration details, including database details.
    RobotDetails: Represents details about a robot.
    JobDetails: Represents details about a job assigned to a robot.
    Route: Represents a route with geographical and topological coordinates.
    RobotLive: Represents live status information of a robot.
    WayPoint: Represents a waypoint in a route.
    SubTasks: Represents a sub-task with a description and data.
    Task: Represents a task assigned to a robot with sub-tasks.
    TopologicalMapData: Represents data related to a topological map.
    RobotLiveStatus: Represents live status information of a robot in string format.
    JobPublisher: Represents a job publisher with a timer.
"""

from dataclasses import dataclass
from typing import Any

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from status_interfaces.msg import SubTask, Task

from status_server.enum import TaskState


@dataclass
class DockStation:
    """
    Represents a row of the `dock_station` table.

    The physical dock pose is owned by the robot's Nav2 docking server and is
    addressed by `dock_id`, so no pose is stored here.

    Attributes:
        id (int): Primary key of the dock station.
        dock_id (str): Dock name registered with the Nav2 docking server.
        type (int): Purpose of the dock, see DockTypeEnum.
        status (int): Occupancy state, see ChargingStationStatusEnum.
        robot_id (int | None): Robot currently holding the dock, if any.
        node_id (int): Farm node the robot must reach before docking.
        dock_type (str): Nav2 dock plugin type, e.g. 'simple_charging_dock'.
    """

    id: int
    dock_id: str
    type: int
    status: int
    robot_id: int | None
    node_id: int
    dock_type: str


@dataclass
class Point:
    """
    Represents a geographical point with latitude and longitude.
    Attributes:
        lat (float): The latitude of the point.
        lon (float): The longitude of the point.
    """

    lat: float
    lon: float


@dataclass
class TopologyMapPosition:
    """
    Represents a position in a topology map with six degrees of freedom.
    Attributes:
        x (float): The x-coordinate of the position.
        y (float): The y-coordinate of the position.
        z (float): The z-coordinate of the position.
        r (float): The roll angle of the position in radians.
        p (float): The pitch angle of the position in radians.
        w (float): The yaw angle of the position in radians.
    """

    x: float
    y: float
    z: float
    r: float
    p: float
    w: float


@dataclass
class Time:
    """
    A class representing time with seconds and nanoseconds.
    Attributes:
        sec (int): The number of seconds.
        nanosec (int): The number of nanoseconds.
    """

    sec: int
    nanosec: int


@dataclass
class FarmAssets:
    """
    A data class representing farm assets.
    Attributes:
        id (int): The unique identifier for the farm asset.
        name (str): The name of the farm asset.
        x_map (float): The x-coordinate of the asset on the map.
        y_map (float): The y-coordinate of the asset on the map.
        conn_nodes (list[int]): A list of connected node identifiers.
    """
    id: int
    name: str
    x_map: float
    y_map: float
    conn_nodes: list[int]


@dataclass
class FarmEdges:
    """Class representing a farm edge."""
    id: int
    zone_prefix: str
    from_node: int
    to_node: int
    distance: float
    weight: float


@dataclass
class FarmNodes:
    """Class representing a farm edge."""
    id: int
    zone_prefix: str
    x: float
    y: float
    type: float
    theta: float


@dataclass
class ClearanceConfig:
    """
    How far waypoints are pushed off the crop before they are published.

    The graph places a Pickup node at the distance the manipulator wants,
    which is closer to the bush than the robot's body can legally sit: Nav2
    marks a band of the robot's own inscribed radius around every mapped
    obstacle as untraversable, and refuses any pose whose footprint touches
    it. Publishing the node position unchanged gives the planner a goal it
    will always reject.

    Shifting the waypoint away from the bush buys that clearance back, and
    costs exactly the same distance in manipulator reach. max_shift is the
    cap: past it the geometry is wrong and should be fixed in the map or the
    graph rather than papered over here.

    Attributes:
        enabled (bool): Apply the shift at all.
        robot_half_width (float): Half the footprint width, which is the
            inscribed radius Nav2 inflates by. Sets how far sideways an object
            must be.
        robot_half_length (float): Half the footprint length. Sets how far
            ahead or behind an object still counts as level with the robot -
            a long robot is threatened by a bush its centre has not reached.
        obstacle_radius (float): How far the mapped obstacle extends from the
            object centre stored in object_data.
        safety_margin (float): Extra metres on top of the two radii.
        max_shift (float): Largest shift allowed for one waypoint. A waypoint
            needing more is published short of the requirement and logged.
        collinear_tolerance (float): How far a waypoint may sit from the
            straight line between its neighbours and still be dropped. A row
            of Pickup nodes is a straight drive, so publishing every node in
            it gives Nav2 twenty goals where two would do.
    """

    enabled: bool
    robot_half_width: float
    robot_half_length: float
    obstacle_radius: float
    safety_margin: float
    max_shift: float
    collinear_tolerance: float

    @property
    def standoff(self) -> float:
        """Distance from an object centre at which the robot is legal."""
        return self.obstacle_radius + self.robot_half_width + self.safety_margin


@dataclass
class GraphNode:
    """
    Represents a row of the `graph_node` table.

    Replaces FarmNodes. There is no zone_prefix: `id` is the sole primary key,
    so a node ID identifies exactly one row.

    Attributes:
        id (int): Primary key, and the ID carried on WayPoint messages.
        node_type (str): Role of the node, see NodeTypeEnum.
        x (float): X-coordinate in the map frame.
        y (float): Y-coordinate in the map frame.
        theta (float): Approach heading in radians. Never NULL in this graph;
            on a Pickup node it also selects which side of the bush the
            manipulator can reach.
        obj_id (int | None): Object this node serves, NULL for nodes that are
            not Pickup nodes.
    """
    id: int
    node_type: str
    x: float
    y: float
    theta: float
    obj_id: int | None


@dataclass
class GraphEdge:
    """
    Represents a row of the `graph_edge` table.

    Replaces FarmEdges. There is no distance column: `weight` is a traversal
    cost, roughly euclidean length times a per-edge-type multiplier, so it
    must not be read as metres. TaskGenerator builds a separate euclidean
    matrix for anything that needs real distance.

    Attributes:
        source_id (int): Node the edge leaves.
        target_id (int): Node the edge enters.
        weight (float): Traversal cost, not metres.
        edge_type (str): Kind of edge, see EdgeTypeEnum.
    """
    source_id: int
    target_id: int
    weight: float
    edge_type: str


@dataclass
class GraphObject:
    """
    Represents a row of the `object_data` table.

    Replaces FarmAssets. The bush itself, not the nodes used to reach it: each
    object is served by exactly two Pickup nodes, one per side of the row.

    Attributes:
        object_id (int): Primary key, unique across the whole farm rather
            than per row.
        row_id (int): Row the object sits in.
        x_coord (float): X-coordinate of the object in the map frame.
        y_coord (float): Y-coordinate of the object in the map frame.
        orientation_deg (float): Object heading in degrees.
    """
    object_id: int
    row_id: int
    x_coord: float
    y_coord: float
    orientation_deg: float


@dataclass
class HarvestUnit:
    """
    One side of one bush - the smallest unit of harvesting work.

    A bush is harvested in two passes because the manipulator reaches only
    half of it from either side, so each object yields two HarvestUnits.
    fetch_harvest_units returns them already in sweep order.

    Attributes:
        name (str): Asset name, 'b{row_id}_{object_id}'. Written to
            farm_harvesting_job.job_schedule and used with node_id to decide
            whether this side is already done.
        node_id (int): Pickup node the robot approaches from.
        row_id (int): Row the bush sits in.
        x (float): X-coordinate of the bush.
        y (float): Y-coordinate of the bush.
    """
    name: str
    node_id: int
    row_id: int
    x: float
    y: float


@dataclass
class RobotDocks:
    """
    Dock IDs one robot is allowed to use, read from the `docks` config block.

    dock_id is globally unique and is both the `dock_station` key and the name
    registered with that robot's Nav2 docking server. Two robots may list the
    same dock; the dock_station reservation decides who gets it.

    Attributes:
        charging (list[str]): Dock IDs usable for charging.
        unloading (list[str]): Dock IDs usable for unloading. Empty until an
            unloading dock exists.
    """
    charging: list[str]
    unloading: list[str]


@dataclass
class Path:
    """Class representing a path in the graph."""
    id: int
    start_node: int
    end_node: int
    distance: float
    path: list[int] | None


@dataclass
class JobSequence:
    """Class representing a job sequence."""
    job_id: int
    job_schedule: str
    conn_nodes: list
    crop_type: str
    harvest_ready: int
    progress_status: int
    assigned_robot_id: int
    job_location: Point


@dataclass
class JobPublishers:
    """
    Represents the publishers and subscribers associated with a robot job.
    Attributes:
        id (int): Unique identifier for the robot job publisher.
        topic_name (str): Name of the topic associated with the publisher.
        topic_publisher (Publisher): Publisher object for the topic.
        topic_subscriber (Subscription): Subscriber object for the topic.
    """

    id: int
    serial_number: str
    topic_name: str
    topic_publisher: Publisher
    topic_timer: Timer


@dataclass
class RobotJobPublishers:
    """
    Represents the publishers and subscribers associated with a robot job.
    Attributes:
        id (int): Unique identifier for the robot job publisher.
        topic_name (str): Name of the topic associated with the publisher.
        topic_publisher (Publisher): Publisher object for the topic.
        topic_subscriber (Subscription): Subscriber object for the topic.
    """

    id: int
    topic_name: str
    topic_publisher: Publisher
    topic_subscriber: Subscription


@dataclass
class DatabaseDetails:
    """
    Class to store database connection details.
    Attributes:
        host (str): The hostname or IP address of the database server.
        port (int): The port number on which the database server is listening.
        dbname (str): The name of the database to connect to.
        user (str): The username for authenticating with the database.
        password (str): The password for authenticating with the database.
        connect_timeout (int): Seconds to wait for a connection before giving
            up. Without it psycopg uses the OS TCP timeout, which can block a
            node for minutes when the host is unreachable.
    """

    host: str
    port: int
    dbname: str
    user: str
    password: str
    connect_timeout: int


@dataclass
class NamespaceConfig:
    """
    NamespaceConfig holds configuration for namespace selection modes.
    Attributes:
        mode (str): Specifies the namespace selection mode.
            Expected values are 'single' or 'multi', as provided in the YAML configuration.
        namespace (str): The namespace to use when mode is set to 'single'.
        namespaces (list[str]): A list of namespaces to use when mode is set to 'multi'.
    """

    mode: str
    namespace: str
    namespaces: list[str]


@dataclass
class BatteryConfig:
    """
    Settings for deciding whether a robot can take on and survive a task.

    Attributes:
        reserve_threshold (float): Minimum battery level, in percent (0-100),
            required before a new task is assigned. Robots at or below this
            level are sent to the charging station instead.
        ewma_alpha (float): Smoothing factor for the running power and speed
            estimates. Higher reacts faster and is noisier.
        safety_factor (float): Margin applied to predicted energy before
            comparing it against the charge actually on board.
        min_move_distance (float): Metres of movement required before a sample
            counts towards the speed estimate.
        fixed_task_seconds (dict[str, float]): Whole-task durations keyed by
            task name, including each task's own navigation. Charging is
            absent by design - it is the fallback task and is never vetoed.
    """

    reserve_threshold: float
    ewma_alpha: float
    safety_factor: float
    min_move_distance: float
    fixed_task_seconds: dict[str, float]


@dataclass
class RobotEnergy:
    """
    Running energy picture for one robot, rebuilt from the live BMS feed.

    Held in memory by the task manager rather than the database: it is the
    process that decides feasibility, and the estimates are cheap to relearn
    after a restart.

    Attributes:
        energy_wh (float | None): Charge remaining, charge (Ah) x voltage (V).
        avg_power_w (float | None): Smoothed draw, voltage x abs(current).
            Every sample counts, so idle, sensors and manipulator are all in.
        avg_speed_mps (float | None): Smoothed travel speed. Only samples
            where the robot actually moved contribute.
        last_x (float | None): Last position used for a speed sample.
        last_y (float | None): Last position used for a speed sample.
        last_time (float | None): Timestamp of that position, in seconds.
    """

    energy_wh: float | None = None
    avg_power_w: float | None = None
    avg_speed_mps: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    last_time: float | None = None

    def readings(self) -> tuple[float, float, float] | None:
        """
        Return (energy_wh, avg_power_w, avg_speed_mps) once all three exist.

        Handing back a tuple rather than letting callers read the optional
        fields directly keeps the "are these populated" check and the use of
        the values in one place, so a caller cannot divide by a speed that was
        never measured.

        Returns:
            The three readings, or None while any is still missing. Speed of
            zero counts as missing: the return-leg prediction divides by it.
        """
        if (self.energy_wh is None
                or self.avg_power_w is None
                or self.avg_speed_mps is None
                or self.avg_speed_mps <= 0.0):
            return None
        return self.energy_wh, self.avg_power_w, self.avg_speed_mps

    def is_ready(self) -> bool:
        """True once there is enough data to predict a task's energy cost."""
        return self.readings() is not None


@dataclass
class FarmConfig:
    """
    Farm-wide settings that are not held in the database.

    Attributes:
        crop_type (str): Crop grown on this farm, written onto harvest tasks.
        graph_source (str): Which table family supplies the routing graph.
            'graph' uses graph_node / graph_edge / object_data, 'farm' uses
            the older farm_node / farm_edge / farm_asset_on_map. The rollback
            switch: the old tables are left in place, so flipping this back
            restores the previous behaviour without a code change.
    """

    crop_type: str
    graph_source: str


@dataclass
class LivenessConfig:
    """
    Watchdog settings for deciding whether a robot is still alive.

    Attributes:
        timeout (float): Seconds without a status message before the robot is
            marked offline. Must exceed the worst-case gap between messages.
        check_period (float): How often the watchdog scans for stale robots.
    """

    timeout: float
    check_period: float


@dataclass
class Config:
    """
    A configuration class for storing application settings.
    Attributes:
        namespace_config (NamespaceConfig): Namespace subscription settings.
        battery (BatteryConfig): Battery threshold settings.
        liveness (LivenessConfig): Robot liveness watchdog settings.
        farm (FarmConfig): Farm-wide settings.
        clearance (ClearanceConfig): How far waypoints are pushed off the crop.
        docks (dict): Raw `docks` block, namespace -> purpose -> list of dock
            IDs. Left untyped here because the keys are robot namespaces;
            Configuration.get_docks() converts it to RobotDocks.
        database (DatabaseDetails): Details of the database configuration.
    """
    namespace_config: NamespaceConfig
    battery: BatteryConfig
    liveness: LivenessConfig
    farm: FarmConfig
    clearance: ClearanceConfig
    docks: dict[str, Any]
    database: DatabaseDetails


@dataclass
class RobotDetails:
    """
    Class representing the details of a robot.
    Attributes:
        id (int): Unique identifier for the robot.
        robot_name (str): Name of the robot.
        robot_type (str): Type or category of the robot.
        robot_description (str): Description of the robot's functionality or purpose.
        manufacturer (str): Name of the manufacturer of the robot.
        serial_number (str): Serial number of the robot for identification.
    """

    id: int
    robot_name: str
    robot_type:  str
    robot_description: str
    manufacturer: str
    serial_number: str


@dataclass
class JobDetails:
    """
    Represents the details of a job in the Robot Status Monitor system.
    Attributes:
        id (int): Unique identifier for the job.
        node_id (int): Identifier for the node associated with the job.
        job_schedule (str): Scheduled time or identifier for the job.
        crop_type (str): Type of crop associated with the job.
        harvest_ready (int): Indicator of whether the crop is ready for harvest.
        progress_status (float): Progress status of the job as a percentage.
        assigned_robot_id (int): Identifier of the robot assigned to the job.
        gps_location (Point): GPS location of the job site.
        task_type (int): Type of task this row represents, see TaskEnum.

    Note:
        Field order matches the column order of `farm_harvesting_job`, because
        rows are unpacked positionally as JobDetails(*row).
    """

    id: int
    node_id: int
    job_schedule: str
    crop_type: str
    harvest_ready: int
    progress_status: float
    assigned_robot_id: int
    gps_location: Point
    task_type: int


@dataclass
class Route:
    """
    Represents a route with geographical and positional information.
    Attributes:
        id (int): Unique identifier for the route.
        node_id (int): Identifier for the associated node.
        zone_id (int): Identifier for the associated zone.
        latitude (float): Latitude coordinate of the route.
        longitude (float): Longitude coordinate of the route.
        x (float): X-coordinate in a Cartesian system.
        y (float): Y-coordinate in a Cartesian system.
    """

    id: int
    node_id: int
    zone_id: int
    latitude: float
    longitude: float
    x: float
    y: float


@dataclass
class RobotLive:
    """
    Class representing the live status of a robot.
    Attributes:
        id (int): Unique identifier for the robot.
        time (str): Timestamp of the status update.
        ros_time (Time): ROS time associated with the status update.
        robot_type (str): Type of the robot.
        online_flag (int): Flag indicating whether the robot is online (1) or offline (0).
        status (str): Current status of the robot.
        gps_location (Point): GPS location of the robot.
        task (str): Current task being performed by the robot.
        topo_map_pos (TopologyMapPosition): Topological map position of the robot.
        current_node_id (int): ID of the current node in the topological map.
        target_node_id (int): ID of the target node in the topological map.
        battery_level (float): Current battery level of the robot as a percentage.
        operation_hours_after_charging (str): Hours of operation since the last charge.
        load_status (float): Current load status of the robot.
        crop_type (str): Type of crop being handled by the robot.
    """

    id: int
    time: str
    ros_time: Time
    robot_type: str
    online_flag: int
    status: int
    gps_location: Point
    task: str
    topo_map_pos: TopologyMapPosition
    current_node_id: int
    target_node_id: int
    battery_level: float
    operation_hours_after_charging: str
    load_status: float
    crop_type: str


@dataclass
class TopologicalMapData:
    """
    A class to represent the topological map data used in the Robot Status Monitor.
    Attributes:
        cost_matrix (Any): The cost matrix representing the costs between nodes.
        num_nodes (int): The total number of nodes in the map.
        relative_job_coords (list[tuple[float, float]]): The relative coordinates of job locations.
        relative_node_coords (list[tuple[float, float]]): The relative coordinates of node location.
        start_id_list (list[int]): A list of start node IDs.
        end_id_list (list[int]): A list of end node IDs.
        cs_locations (list[tuple[float, float]]): The coordinates of charging station locations.
        node_locations (list[tuple[float, float]]): The coordinates of all node locations.
        us_locations (list[tuple[float, float]]): The coordinates of user station locations.
        lane_info (list[int]): Information about lanes in the map.
        num_lanes (int): The total number of lanes in the map.
        num_tasks_per_lane (int): The number of tasks assigned per lane.
        us_dx (float): The x-axis offset for user stations.
        us_dy (float): The y-axis offset for user stations.
    """

    cost_matrix: Any
    num_nodes: int
    relative_job_coords: list[tuple[float, float]]
    relative_node_coords: list[tuple[float, float]]
    start_id_list: list[int]
    end_id_list: list[int]
    cs_locations: list[tuple[float, float]]
    node_locations: list[tuple[float, float]]
    us_locations: list[tuple[float, float]]
    lane_info: list[int]
    num_lanes: int
    num_tasks_per_lane: int
    us_dx: float
    us_dy: float


@dataclass
class RobotLiveStatus:
    """
    RobotLiveStatus class represents the live status of a robot with various attributes.
    Attributes:
        id (int): Unique identifier for the robot.
        time (str): Timestamp of the status update.
        ros_time (str): ROS-specific timestamp of the status update.
        robot_type (str): Type or model of the robot.
        online_flag (int): Flag indicating whether the robot is online (1) or offline (0).
        status (str): Current status of the robot.
        gps_location (str): GPS coordinates of the robot's current location.
        task (str): Current task assigned to the robot.
        topo_map_pos (str): Topological map position of the robot.
        current_node_id (int): Identifier of the current node in the map.
        target_node_id (int): Identifier of the target node in the map.
        battery_level (float): Current battery level of the robot as a percentage.
        operation_hours_after_charging (str): Hours of operation since the last charge.
        load_status (float): Current load status of the robot.
        crop_type (str): Type of crop the robot is handling, if applicable.
    """

    id: int
    time: str
    ros_time: str
    robot_type: str
    online_flag: int
    status: str
    gps_location: str
    task: str
    topo_map_pos: str
    current_node_id: int
    target_node_id: int
    battery_level: float
    operation_hours_after_charging: str
    load_status: float
    crop_type: str


@dataclass
class Nodes:
    """
    Represents a node with geographical and positional information.
    Attributes:
        id (int): Unique identifier for the node.
        zone_id (int): Identifier for the zone to which the node belongs.
        latitude (float): Latitude coordinate of the node.
        longitude (float): Longitude coordinate of the node.
        x (float): X-coordinate of the node in a Cartesian plane.
        y (float): Y-coordinate of the node in a Cartesian plane.
    """
    id: int
    zone_id: int
    latitude: float
    longitude: float
    x: float
    y: float


@dataclass
class RobotTaskState:
    """
    Complete state tracking for each robot

    This class maintains all necessary information about a robot's current
    state, task execution, and performance metrics.
    """
    # Task information
    current_task: Task | None = None
    task_state: TaskState = TaskState.NOT_STARTED
    current_subtask_index: int = 0
    last_published_subtask: SubTask | None = None

    # Status tracking
    last_reported_status: int | None = None
    previous_status: int | None = None
    status_update_count: int = 0

    # Timing information
    task_start_time: float | None = None
    last_status_change_time: float | None = None
    subtask_start_time: float | None = None

    # Error tracking
    error_count: int = 0
    consecutive_same_status_count: int = 0

    # Performance metrics
    completed_tasks_count: int = 0
    failed_tasks_count: int = 0
    total_task_time: float = 0.0

    # Health flags
    is_stuck: bool = False
    requires_intervention: bool = False

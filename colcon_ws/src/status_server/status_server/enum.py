"""
This module defines various enumerations used in the Robot Status Monitor system.

Enumerations:
    RobotType: Enumerates different types of robots.
    OnlineFlagEnum: Enumerates the online status of a robot.
    RobotStatusEnum: Enumerates various statuses a robot can have.
    LoadStatus: Enumerates the load status of a robot.
    HarvestReadyStatus: Enumerates the harvest readiness status.
    ChargingStationStatus: Enumerates the status of a charging station.
    ChargingStatus: Enumerates the charging status of a robot.
"""

from enum import Enum


class RobotTypeEnum(Enum):
    """
    Enum class representing different types of robots.
    Attributes:
        HUSKY (int): Represents the Husky robot type with a value of 0.
        JACKAL (int): Represents the Jackal robot type with a value of 1.
    """

    HUSKY = 0
    JACKAL = 1


class TaskEnum(Enum):
    """
    Enumeration representing different types of tasks for the system.
    Attributes:
        CHARGING_TASK (int): Represents a charging task.
        HARVESTING_TASK (int): Represents a harvesting task.
        UNLOADING_TASK (int): Represents an unloading task.
    """

    CHARGING_TASK = 0
    HARVESTING_TASK = 1
    UNLOADING_TASK = 2


class SubTaskEnum(Enum):
    """
    SubTaskEnum is an enumeration that represents various sub-tasks
    that can be performed in the system. Each sub-task is associated
    with a unique integer value.
    Attributes:
        MOVING (int): Represents the task of moving.
        HARVESTING (int): Represents the task of harvesting.
        DOCKING (int): Represents the task of docking.
        CHARGING (int): Represents the task of charging.
        LOADING (int): Represents the task of loading.
        UNLOADING (int): Represents the task of unloading.
    """

    MOVING = 1
    HARVESTING = 2
    DOCKING = 3
    CHARGING = 4
    LOADING = 5
    UNLOADING = 6
    UNDOCKING = 7


class DockTypeEnum(Enum):
    """
    Enum class representing the purpose of a dock station.

    Maps to the `type` column of the `dock_station` table.

    Attributes:
        CHARGING (int): Dock used for battery charging.
        UNLOADING (int): Dock used for unloading harvested crop.
    """

    CHARGING = 0
    UNLOADING = 1


class NodeTypeEnum(Enum):
    """
    Enum class representing the role of a graph node.

    Maps to the `node_type` column of the `graph_node` table.

    Attributes:
        PICKUP (str): Approach point beside a bush. Carries an obj_id, and its
            theta selects which side of the bush the manipulator reaches.
        VIA_CW (str): Ring node, clockwise direction of travel.
        VIA_CCW (str): Ring node, counter-clockwise direction of travel.
        ENTERING (str): Node a row lane starts from.
        EXITING (str): Node a row lane ends at.
        STAGING (str): Parking or charging node off the ring.
    """

    PICKUP = 'Pickup'
    VIA_CW = 'Via_CW'
    VIA_CCW = 'Via_CCW'
    ENTERING = 'Entering'
    EXITING = 'Exiting'
    STAGING = 'Staging'


class EdgeTypeEnum(Enum):
    """
    Enum class representing the kind of a graph edge.

    Maps to the `edge_type` column of the `graph_edge` table. The type sets
    the multiplier between an edge's euclidean length and its weight, so
    weight is a routing cost and never a distance in metres.

    Attributes:
        PICKUP (str): Bush to bush along a row lane.
        TRANSIT (str): Ring node to ring node.
        VIA_SWAP (str): Zero-length hop between the CW and CCW rings.
        VIA_CW (str): Ring edge, clockwise.
        VIA_CCW (str): Ring edge, counter-clockwise.
        ENTERING (str): Ring into the first bush of a lane.
        EXITING (str): Last bush of a lane back out to the ring.
        STAGING (str): Ring to a staging or charging node.
    """

    PICKUP = 'Pickup Edge'
    TRANSIT = 'Transit Edge'
    VIA_SWAP = 'Via Swap'
    VIA_CW = 'Via Edge CW'
    VIA_CCW = 'Via Edge CCW'
    ENTERING = 'Entering Edge'
    EXITING = 'Exiting Edge'
    STAGING = 'Staging'


class OnlineFlagEnum(Enum):
    """
    OnlineFlagEnum is an enumeration that represents the online status of a robot.
    Attributes:
        OFFLINE (int): Represents the offline state of the robot (value: 0).
        ONLINE (int): Represents the online state of the robot (value: 1).
        EMERGENCY_STOP (int): Represents the emergency stop state of the robot (value: 10).
        ABNORMAL (int): Represents an abnormal state of the robot (value: 11).
    """

    OFFLINE = 0
    ONLINE = 1
    EMERGENCY_STOP = 10
    ABNORMAL = 11


class RobotStatusEnum(Enum):
    """
    RobotStatusEnum is an enumeration that defines various statuses for a robot's
    operation. Each status is represented by a unique integer value.
    Attributes:
        IDLE (int): The robot is idle and not performing any task.
        JOB_START (int): The robot has started a job.
        JOB_DONE (int): The robot has completed a job.
        START_MOVING (int): The robot has started moving.
        MOVING (int): The robot is currently moving.
        DESTINATION_REACHED (int): The robot has reached its destination.
        START_HARVESTING (int): The robot has started the harvesting process.
        HARVESTING (int): The robot is currently harvesting.
        DONE_HARVESTING (int): The robot has completed the harvesting process.
        START_DOCKING (int): The robot has started the docking process.
        DOCKING (int): The robot is currently docking.
        DONE_DOCKING (int): The robot has completed the docking process.
        START_LOADING (int): The robot has started the loading process.
        LOADING (int): The robot is currently loading.
        DONE_LOADING (int): The robot has completed the loading process.
        START_UNLOADING (int): The robot has started the unloading process.
        UNLOADING (int): The robot is currently unloading.
        DONE_UNLOADING (int): The robot has completed the unloading process.
        START_CHARGING (int): The robot has started the charging process.
        CHARGING (int): The robot is currently charging.
        DONE_CHARGING (int): The robot has completed the charging process.
        ERROR (int): The robot has encountered an error.
        PAUSED (int): The robot is paused.
        MAINTENANCE (int): The robot is under maintenance.
        OFFLINE (int): The robot is offline.
        EMERGENCY_STOP (int): The robot has been stopped due to an emergency.
        ABNORMAL (int): The robot is in an abnormal state.
    """

    IDLE = 0

    JOB_START = 1
    JOB_DONE = 2

    START_MOVING = 3
    MOVING = 4
    DESTINATION_REACHED = 5

    START_HARVESTING = 6
    HARVESTING = 7
    DONE_HARVESTING = 8

    START_DOCKING = 9
    DOCKING = 10
    DONE_DOCKING = 11

    START_LOADING = 12
    LOADING = 13
    DONE_LOADING = 14

    START_UNDOCKING = 15
    UNDOCKING = 16
    DONE_UNDOCKING = 17

    START_UNLOADING = 18
    UNLOADING = 19
    DONE_UNLOADING = 20

    START_CHARGING = 21
    CHARGING = 22
    DONE_CHARGING = 23

    CANCLE = 93
    ERROR = 94
    PAUSED = 95
    MAINTENANCE = 96
    OFFLINE = 97
    EMERGENCY_STOP = 98
    ABNORMAL = 99


class LoadStatusEnum(Enum):
    """
    Enum class representing the load status of a robot.
    Attributes:
        EMPTY (int): Indicates that the robot is empty.
        LOADED (int): Indicates that the robot is fully loaded.
        UNLOADED (int): Indicates that the robot has been unloaded.
        LOADING (int): Indicates that the robot is in the process of loading.
        UNLOADING (int): Indicates that the robot is in the process of unloading.
    """

    EMPTY = 0
    LOADED = 1
    UNLOADED = 2
    LOADING = 3
    UNLOADING = 4


class ProgressStatusEnum(Enum):
    """
    Enum class representing the progress status of a task.
    Attributes:
        IDLE (int): Indicates that the task is idle and not started.
        IN_PROGRESS (int): Indicates that the task is currently in progress.
        PAUSED (int): Indicates that the task is paused.
        COMPLETED (int): Indicates that the task is completed.
    """

    IDLE = 0
    IN_PROGRESS = 1
    PAUSED = 2
    COMPLETED = 3


class HarvestReadyStatusEnum(Enum):
    """
    HarvestReadyStatus is an enumeration that represents the readiness status
    of a harvesting process.
    Attributes:
        NOT_READY (int): Indicates that the harvesting process is not ready.
        READY (int): Indicates that the harvesting process is ready.
    """

    NOT_READY = 0
    READY = 1
    # NOT_STARTED = 2
    # IN_PROGRESS = 3
    # PAUSED = 4
    # COMPLETED = 5


class ChargingStationStatusEnum(Enum):
    """
    Enum class representing the status of a charging station.
    Attributes:
        IDLE (int): The charging station is idle and not in use.
        CHARGING (int): The charging station is actively charging a device.
        RESERVED (int): The charging station is reserved for future use.
        OCCUPIED (int): The charging station is occupied but not actively charging.
    """

    IDLE = 0
    CHARGING = 1
    RESERVED = 2
    OCCUPIED = 3


class ChargingStatusEnum(Enum):
    """
    Enum class representing the charging status of a robot.
    Attributes:
        NOT_CHARGING (int): Indicates that the robot is not charging (value: 0).
        CHARGING (int): Indicates that the robot is currently charging (value: 1).
        CHARGED (int): Indicates that the robot is fully charged (value: 2).
        DISCHARGING (int): Indicates that the robot is discharging (value: 3).
        CHARGING_ERROR (int): Indicates that there is an error in the charging process (value: 4).
    """

    NOT_CHARGING = 0
    CHARGING = 1
    CHARGED = 2
    DISCHARGING = 3
    CHARGING_ERROR = 4


class TaskState(Enum):
    """
    Task lifecycle states for tracking task progress

    Attributes:
        NOT_STARTED: Task has been created but not assigned
        PUBLISHED: Task has been sent to robot
        IN_PROGRESS: Robot has acknowledged and is executing task
        COMPLETED: Task finished successfully
        FAILED: Task failed and cannot be completed
        CANCELLED: Task was cancelled by operator or system
    """
    NOT_STARTED = 0
    PUBLISHED = 1
    IN_PROGRESS = 2
    COMPLETED = 3
    FAILED = 4
    CANCELLED = 5

"""
Module: Configuration.

This module provides the `Configuration` class, which is responsible for loading
and parsing database configuration details from a YAML file. It uses the `Config`
dataclass to represent the configuration and provides methods to access the
configuration and database details.
"""
import os
import traceback

import yaml
from ament_index_python import get_package_share_directory
from rclpy.impl.rcutils_logger import RcutilsLogger

from status_server.data_utils import DataUtils
from status_server.dataclass import (
    BatteryConfig,
    ClearanceConfig,
    Config,
    DatabaseDetails,
    FarmConfig,
    LivenessConfig,
    NamespaceConfig,
    RobotDocks,
)


class Configuration:
    """
    Configuration class for loading and managing application settings.

    This class is responsible for loading a YAML configuration file, parsing its
    contents, and converting it into a structured dataclass instance. It provides
    methods to access the overall configuration and specific sections, such as
    database details.

    The parsed result is cached on the class after the first load. Several
    objects construct a Configuration during start-up - each node, each
    database connection, the task generator - and without the cache every one
    of them re-read and re-parsed the same file from disk.

    Consequence: the file is read once per process, so editing config.yaml
    while a node is running has no effect. Nothing reloads config at runtime,
    so this costs nothing today.
    """

    _config: Config | None = None

    def __init__(self):
        """Initialize the configuration, loading from YAML only on first use."""
        self.logger = RcutilsLogger(self.__class__.__name__)

        if Configuration._config is not None:
            self.config = Configuration._config
            return

        try:
            # Construct the full file path using the shared directory and file name
            yaml_file = os.path.join(
                get_package_share_directory('status_server'),
                'config',
                'config.yaml')

            # Load YAML file
            with open(yaml_file, 'r', encoding="utf-8") as file:
                data = yaml.safe_load(file)

            # Convert the loaded dictionary into a Config dataclass instance
            self.config = DataUtils.from_dict(Config, data)
            Configuration._config = self.config

            # logger.info("Configuration loaded successfully.")
        except FileNotFoundError as e:
            self.logger.error(f"Stack Trace: {e}\n {traceback.format_exc()}")

    @classmethod
    def reset_cache(cls) -> None:
        """Drop the cached config so the next instance re-reads the file."""
        cls._config = None

    def get_config(self) -> Config:
        """Return the entire configuration."""
        return self.config

    def get_namespace_config(self) -> NamespaceConfig:
        """Retrieve the namespace configuration."""
        return self.config.namespace_config

    def get_battery_config(self) -> BatteryConfig:
        """Retrieve the battery threshold configuration."""
        return self.config.battery

    def get_liveness_config(self) -> LivenessConfig:
        """Retrieve the robot liveness watchdog configuration."""
        return self.config.liveness

    def get_farm_config(self) -> FarmConfig:
        """Retrieve the farm-wide configuration."""
        return self.config.farm

    def get_clearance_config(self) -> ClearanceConfig:
        """Retrieve the waypoint clearance settings."""
        return self.config.clearance

    def get_graph_source(self) -> str:
        """
        Which table family is live: 'graph' or 'farm'.

        One switch has to move everything, not just routing. Node IDs are not
        shared between the two schemas - node 12 in farm_node is a different
        place from node 12 in graph_node - so a dock that is correct under one
        source points somewhere meaningless under the other.
        """
        return getattr(self.config.farm, 'graph_source', 'graph')

    def get_docks(self, source: str | None = None) -> dict[str, RobotDocks]:
        """
        Return the docks each robot namespace is allowed to use.

        The config lists dock IDs only. Everything else about a dock - its
        node, its Nav2 plugin type, whether it is currently reserved - stays in
        the dock_station table, so a robot gaining a second dock is a config
        edit rather than a schema change.

        A dock ID is global: it is both the dock_station key and the name
        registered with that robot's Nav2 docking server. Two robots may list
        the same ID, in which case the dock_station reservation decides who
        gets it.

        The block is keyed by graph source first, because a dock's node ID only
        means anything within one schema. A flat block with namespaces at the
        top level is still accepted and used for every source, which is what
        older configs look like.

        Args:
            source: Graph source to read. Defaults to the live one.

        Returns:
            Namespace -> RobotDocks. Empty if the block is missing.
        """
        raw = getattr(self.config, 'docks', None) or {}
        source = source or self.get_graph_source()

        # Namespaces start with '/', graph sources do not. That tells the two
        # layouts apart without needing a version field in the file.
        nested = any(not str(key).startswith('/') for key in raw)

        if nested:
            block = raw.get(source)
            if block is None:
                self.logger.error(
                    f"No docks configured for graph_source '{source}'. "
                    f'Known sources in config: {sorted(raw)}.'
                )
                return {}
        else:
            block = raw

        docks: dict[str, RobotDocks] = {}
        for namespace, purposes in (block or {}).items():
            purposes = purposes or {}
            docks[namespace] = RobotDocks(
                charging=list(purposes.get('charging') or []),
                unloading=list(purposes.get('unloading') or []),
            )

        return docks

    def get_robot_docks(self, namespace: str) -> RobotDocks:
        """
        Return one robot's docks, or an empty set if it has none configured.

        The leading slash is optional on both sides. Config keys are written as
        ROS namespaces ('/j100_0921') while the rest of this package works in
        robot_info.serial_number ('j100_0921'), and either spelling resolves to
        the same robot.

        Args:
            namespace: Robot namespace or serial number.
        """
        docks = self.get_docks()
        bare = namespace.lstrip('/')

        entry = docks.get(namespace) or docks.get(bare) or docks.get(f'/{bare}')

        if entry is None:
            self.logger.error(f"No docks configured for '{namespace}'.")
            return RobotDocks(charging=[], unloading=[])

        return entry

    def get_database(self) -> DatabaseDetails:
        """Return the database configuration details."""
        return self.config.database

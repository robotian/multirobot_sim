import traceback
from collections.abc import Mapping, Sequence
from typing import Any, Literal, TypeAlias

import psycopg
from psycopg import DatabaseError, Error, InterfaceError, OperationalError, sql
from rclpy.impl.rcutils_logger import RcutilsLogger

from status_server.configuration import Configuration
from status_server.dataclass import (
    DockStation,
    FarmAssets,
    FarmEdges,
    FarmNodes,
    GraphEdge,
    GraphNode,
    HarvestUnit,
    JobDetails,
    RobotDetails,
    RobotLiveStatus,
)
from status_server.enum import (
    ChargingStationStatusEnum,
    OnlineFlagEnum,
    ProgressStatusEnum,
    RobotStatusEnum,
    TaskEnum,
)

# Query: TypeAlias = LiteralString | bytes | "sql.SQL" | "sql.Composed"
Params: TypeAlias = Sequence[Any] | Mapping[str, Any]


class PostgresConnectionManager:
    """
    Unified PostgreSQL database operations for ROS2 nodes with persistent connection.
    """

    def __init__(self):
        self.logger = RcutilsLogger(self.__class__.__name__)
        config = Configuration()
        self.db_config = config.get_database()
        self.conn: psycopg.Connection | None = None

        if not self._connect():
            self.logger.error('Failed to establish initial database connection')

    def _connect(self) -> bool:
        """
        Establish a PostgreSQL connection.

        connect_timeout is mandatory: without it psycopg falls back to the OS
        TCP timeout, so an unreachable database host blocks the calling thread
        for minutes. Every node here calls the database from its executor
        thread, so that stalls the whole node.
        """
        try:
            self.conn = psycopg.connect(
                host=self.db_config.host,
                port=self.db_config.port,
                dbname=self.db_config.dbname,
                user=self.db_config.user,
                password=self.db_config.password or None,  # None: libpq falls back to PGPASSWORD / ~/.pgpass
                connect_timeout=self.db_config.connect_timeout,
            )
            self.conn.autocommit = True
            return True
        except Error as e:
            self.logger.error(f'Failed to connect to database: {e}\n{traceback.format_exc()}')
            self.conn = None
            return False

    def _has_connection(self) -> bool:
        """Open a connection if there isn't a usable one already."""
        if self.conn is not None and not self.conn.closed:
            return True
        return self._connect()

    def _reconnect(self, reason: Exception) -> bool:
        """Drop a broken connection and open a fresh one."""
        self.logger.warning(f'Database connection lost ({reason}), reconnecting...')
        try:
            if self.conn is not None and not self.conn.closed:
                self.conn.close()
        except (Error, OSError):
            pass
        return self._connect()

    def close(self):
        """Close the database connection."""
        try:
            if self.conn and not self.conn.closed:
                self.conn.close()
                self.logger.info('Database connection closed.')
        except Error as e:
            self.logger.error(f'Error closing DB connection: {e}\n{traceback.format_exc()}')
        finally:
            self.conn = None

    # ----------------- Unified Execution Methods -----------------

    def execute_write(self, query, params: Params | None = None, many: bool = False) -> int | None:
        """
        Execute INSERT, UPDATE or DELETE and report how many rows changed.

        Distinguishes "the statement ran but matched nothing" from "the
        statement failed", which execute_query cannot express. Callers that
        need to know a row was actually written should use this.

        Args:
            query:  SQL statement.
            params: Query parameters, or a list of tuples when many=True.
            many:   Use executemany.

        Returns:
            Number of rows affected, or None if the statement failed.

        Note:
            The statement is sent straight to the server. A broken connection
            surfaces as an error here, which triggers one reconnect and one
            retry - cheaper than probing the connection before every call.
        """
        for attempt in (1, 2):
            if not self._has_connection() or self.conn is None:
                return None

            try:
                with self.conn.cursor() as cur:
                    if many:
                        if not isinstance(params, list):
                            raise ValueError('params must be a list of tuples when many=True')
                        cur.executemany(query, params)
                    else:
                        cur.execute(query, params)
                    if not self.conn.autocommit:
                        self.conn.commit()
                    return cur.rowcount

            except (OperationalError, InterfaceError) as e:
                # Connection-level failure: worth one reconnect and retry.
                if attempt == 1 and self._reconnect(e):
                    continue
                self.logger.error(f'Query execution failed: {e}')
                return None

            except Error as e:
                # Statement-level failure. Retrying would fail identically.
                self.logger.error(f'Query execution failed: {e}\n{traceback.format_exc()}')
                if self.conn and not self.conn.closed:
                    self.conn.rollback()
                return None

        return None

    def execute_query(self, query, params: Params | None = None, many: bool = False) -> bool:
        """
        Execute INSERT, UPDATE, DELETE, or parameterized queries.

        Reports only whether the statement ran, not whether it matched any
        rows. Use execute_write when that distinction matters.
        """
        return self.execute_write(query, params, many) is not None

    def fetch_data(
        self,
        query,
        params: Params | None = None,
        fetch_type: Literal['one', 'many', 'all'] = 'all',
        num_rows: int | None = None,
    ) -> tuple | list[tuple] | None:
        """
        Execute a SELECT query and fetch data based on specified fetch type.

        Note:
            Sent straight to the server. A broken connection surfaces as an
            error here and triggers one reconnect and one retry, instead of
            probing the connection before every query.
        """
        if fetch_type not in ('one', 'many', 'all'):
            raise ValueError("fetch_type must be 'one', 'many', or 'all'")

        if fetch_type == 'many' and num_rows is None:
            raise ValueError("num_rows must be specified for fetch_type='many'")

        for attempt in (1, 2):
            if not self._has_connection() or self.conn is None:
                return None

            try:
                with self.conn.cursor() as cur:
                    cur.execute(query, params)
                    if fetch_type == 'one':
                        return cur.fetchone()
                    if fetch_type == 'many':
                        return cur.fetchmany(num_rows)
                    return cur.fetchall()

            except (OperationalError, InterfaceError) as e:
                if attempt == 1 and self._reconnect(e):
                    continue
                self.logger.error(f'Data fetch failed: {e}')
                return None

            except Error as e:
                self.logger.error(f'Data fetch failed: {e}\n{traceback.format_exc()}')
                return None

        return None

    def execute_transaction(self, queries: list) -> bool:
        """
        Execute multiple queries in a single transaction.
        queries: list of query strings or (query, params) tuples.
        Note: This temporarily disables autocommit to execute as a transaction.
        """
        original_autocommit = None
        try:
            if not self._has_connection() or self.conn is None:
                raise Error('Unable to connect to database.')

            # Temporarily disable autocommit for transaction
            original_autocommit = self.conn.autocommit
            if original_autocommit:
                self.conn.autocommit = False

            with self.conn.cursor() as cur:
                for q in queries:
                    if isinstance(q, tuple):
                        query, params = q
                        cur.execute(query, params)
                    else:
                        cur.execute(q)
                self.conn.commit()
            return True
        except Error as e:
            self.logger.error(f'Transaction failed, rolling back: {e}\n{traceback.format_exc()}')
            if self.conn:
                self.conn.rollback()
            return False
        finally:
            # Restore original autocommit setting
            if original_autocommit is not None and self.conn:
                self.conn.autocommit = original_autocommit

    # ----------------- Utility Methods -----------------

    def table_exists(self, table_name: str, schema: str = 'public') -> bool:
        """Check if a table exists."""
        query = """
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_schema = %s AND table_name = %s
            )
        """
        result = self.fetch_data(query, (schema, table_name), fetch_type='one')

        self.logger.debug(f'Table exists: {result}')

        return bool(result[0]) if result else False

    def row_exists(self, table_name: str, row_id: int) -> bool:
        """Check if a row exists."""
        query = sql.SQL('SELECT EXISTS (SELECT FROM {table} WHERE id = %s);').format(table=sql.Identifier(table_name))

        result = self.fetch_data(query, (row_id,), fetch_type='one')

        self.logger.debug(f'Row with ID = {row_id} exists in table {table_name}: {result}')

        return bool(result[0]) if result else False

    def get_row_count(self, table_name: str, where: dict[str, Any] | None = None) -> int | None:
        """
        Get the number of rows in a table, optionally filtered by one or more conditions.

        Args:
            table_name (str): Name of the table.
            where (dict[str, Any], optional): Dictionary of column=value pairs for the WHERE clause.
                If None or empty, counts all rows in the table.

        Returns:
            int | None: Number of rows matching the condition, or None if the query fails.

        Examples:
            # Count all rows in the table
            total_rows = self.db.get_row_count("robot_info")

            # Count rows where id = 1
            rows_with_id = self.db.get_row_count("robot_info", where={"id": 1})

            # Count rows with multiple conditions
            rows_filtered = self.db.get_row_count(
                "robot_info",
                where={"status": "active", "type": "robot"}
            )
        """
        try:
            if not self._has_connection() or self.conn is None:
                return None

            base_query = sql.SQL('SELECT COUNT(*) FROM {table}').format(table=sql.Identifier(table_name))

            params: list[Any] = []
            if where:
                # Build safe WHERE clause
                conditions = []
                for col, val in where.items():
                    conditions.append(sql.SQL('{col} = %s').format(col=sql.Identifier(col)))
                    params.append(val)
                where_clause = sql.SQL(' WHERE ').join([sql.SQL(''), sql.SQL(' AND ').join(conditions)])
                query = sql.Composed([base_query, where_clause])
            else:
                query = base_query

            result = self.fetch_data(query=query, params=tuple(params) if params else None, fetch_type='one')
            if result and len(result) > 0:
                count = result[0]
                if isinstance(count, (int, float, str)):
                    return int(count)
            return None
        except (Error, ValueError, TypeError) as e:
            self.logger.error(f'Error fetching row count: {e}')
            return None

    def is_connected(self) -> bool:
        """Check if database connection is active."""
        try:
            if self.conn and not self.conn.closed:
                with self.conn.cursor() as cur:
                    cur.execute('SELECT 1')
                return True
        except (Error, OSError) as e:
            self.logger.debug(f'Connection liveness check failed: {e}')
        return False

    def get_connection_info(self) -> dict | None:
        """Get info about current connection."""
        if self.conn:
            info = self.conn.info
            return {
                'host': self.db_config.host,
                'port': self.db_config.port,
                'database': self.db_config.dbname,
                'user': self.db_config.user,
                'status': 'connected' if not self.conn.closed else 'closed',
                'server_version': info.server_version if not self.conn.closed else None,
            }
        return None


class PostgresOperations:
    """
    Enhanced database operations for task manager using SafePostgresClient.
    Handles robot status, task updates, and node information with safe queries.
    """

    def __init__(self, db: PostgresConnectionManager | None = None):
        """
        Args:
            db: Connection manager to use. Pass an existing one to share a
                single socket across collaborating objects; omit it to open a
                new connection.
        """
        self.db = db if db is not None else PostgresConnectionManager()
        self.logger = RcutilsLogger(self.__class__.__name__)

        # Which table family the map dispatchers read. Resolved once: the
        # config is not reloadable at runtime, and every path lookup would
        # otherwise re-read it.
        self.graph_source = Configuration().get_farm_config().graph_source
        self._use_graph_tables = self.graph_source != 'farm'

        if not self._use_graph_tables:
            self.logger.warning("graph_source is 'farm': routing from the legacy farm_node / farm_edge tables.")

        # Caches for farm geometry only. Nothing in this package writes
        # farm_node or farm_asset_on_map, and farm_edge is already frozen at
        # start-up inside TaskGenerator's distance matrix. Live tables -
        # robot_live, farm_harvesting_job, dock_station - are never cached.
        # Call clear_cache() after editing the farm layout.
        self._nodes_cache: list[FarmNodes] | None = None
        self._navigable_nodes_cache: list[FarmNodes] | None = None
        self._assets_cache: list[FarmAssets] | None = None

        # Same treatment for the graph_* tables that replace them.
        self._graph_nodes_cache: list[GraphNode] | None = None
        self._navigable_graph_nodes_cache: list[GraphNode] | None = None
        self._harvest_units_cache: list[HarvestUnit] | None = None
        self._object_positions_cache: list[tuple[float, float]] | None = None

    def clear_cache(self) -> None:
        """Drop cached farm geometry so the next read hits the database."""
        self._nodes_cache = None
        self._navigable_nodes_cache = None
        self._assets_cache = None
        self._graph_nodes_cache = None
        self._navigable_graph_nodes_cache = None
        self._harvest_units_cache = None
        self._object_positions_cache = None

    # ---------------- Robot Status ----------------

    def fetch_robot_status(self, robot_id: int) -> RobotLiveStatus | None:
        """
        Fetch current robot status from database.

        Args:
            robot_id: ID of the robot

        Returns:
            RobotLiveStatus object or None if not found
        """
        try:
            row = self.db.fetch_data('SELECT * FROM robot_live WHERE id = %s', params=(robot_id,), fetch_type='one')

            if row and isinstance(row, tuple):
                return RobotLiveStatus(*row)
            elif isinstance(row, list) and len(row) > 0 and isinstance(row[0], tuple):
                self.logger.warning(f'Unexpected data type from database: {type(row)}')
                self.logger.warning(f'Unexpected data from database: {row}')
                return RobotLiveStatus(*row[0])  # use only the first record

            return None

        except DatabaseError as e:
            self.logger.error(f'Database error fetching robot status: {e}')
            return None

    # ---------------- Task Progress ----------------

    def fetch_open_task(self, robot_id: int) -> JobDetails | None:
        """
        Fetch the robot's currently open task, whatever its type.

        An open task is any row not yet marked complete, i.e. progress_status
        in (NOT STARTED, IN PROGRESS, PAUSED). This is the single source used
        to rehydrate a robot's task after a server restart, replacing the old
        status-group reconstruction logic.

        The newest open row wins. A robot interrupted by low battery has both
        its paused task and its newer charging task open at once, and the
        charging task is the one it is actually executing.

        Args:
            robot_id: Database ID of the robot.

        Returns:
            JobDetails for the newest open task, or None if the robot has none.
        """
        try:
            query = """
                SELECT * FROM public.farm_harvesting_job
                WHERE assigned_robot_id = %s
                  AND progress_status IN (%s, %s, %s)
                ORDER BY id DESC
                LIMIT 1
            """
            row = self.db.fetch_data(
                query,
                params=(
                    robot_id,
                    ProgressStatusEnum.IDLE.value,
                    ProgressStatusEnum.IN_PROGRESS.value,
                    ProgressStatusEnum.PAUSED.value,
                ),
                fetch_type='one',
            )

            if row and isinstance(row, tuple):
                return JobDetails(*row)
            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching open task for robot {robot_id}: {e}')
            return None

    def insert_task(
        self,
        node_id: int,
        job_schedule: str,
        crop_type: str,
        harvest_ready: int,
        assigned_robot_id: int,
        location: tuple[float, float],
        task_type: int,
    ) -> int | None:
        """
        Insert a new task row and return its generated ID.

        `farm_harvesting_job.id` is an identity column, so IDs are sequential
        across all task types. The returned ID becomes Task.task_id, which is
        why the insert must report it back rather than returning a bool.

        Args:
            node_id: Farm node the robot must reach for this task.
            job_schedule: Label for the target - bush asset name for harvest
                tasks, dock_id for charging and unloading tasks.
            crop_type: Crop being handled, empty string for dock tasks.
            harvest_ready: 1 for harvest tasks, 0 for dock tasks.
            assigned_robot_id: Robot the task is assigned to.
            location: (x, y) of the target, stored in the job_location point.
            task_type: See TaskEnum.

        Returns:
            The new row's ID, or None if the insert failed.
        """
        try:
            query = """
                INSERT INTO public.farm_harvesting_job (
                    node_id, job_schedule, crop_type, harvest_ready,
                    progress_status, assigned_robot_id, job_location, task_type
                ) VALUES (%s, %s, %s, %s, %s, %s, Point(%s, %s), %s)
                RETURNING id
            """
            row = self.db.fetch_data(
                query,
                params=(
                    node_id,
                    job_schedule,
                    crop_type,
                    harvest_ready,
                    ProgressStatusEnum.IDLE.value,
                    assigned_robot_id,
                    location[0],
                    location[1],
                    task_type,
                ),
                fetch_type='one',
            )

            if row and isinstance(row, tuple):
                task_id = int(row[0])
                self.logger.info(
                    f'Created task id={task_id} type={task_type} '
                    f'schedule={job_schedule} node={node_id} '
                    f'robot={assigned_robot_id}'
                )
                return task_id

            self.logger.error(f'Task insert returned no id: schedule={job_schedule} node={node_id}')
            return None

        except DatabaseError as e:
            self.logger.error(f'Error inserting task: {e}')
            return None

    def task_exists(self, job_schedule: str, node_id: int) -> bool:
        """
        Check whether a task already exists for a target.

        A harvest target is one side of one bush, identified by the pair
        (job_schedule, node_id) - the two conn_nodes of a bush share a name
        but have different approach nodes. Used to skip targets already done.

        Args:
            job_schedule: Target label (bush asset name or dock_id).
            node_id: Approach node for that target.

        Returns:
            True if any row already covers this target.
        """
        try:
            query = """
                SELECT EXISTS (
                    SELECT 1 FROM public.farm_harvesting_job
                    WHERE job_schedule = %s AND node_id = %s
                )
            """
            row = self.db.fetch_data(query, params=(job_schedule, node_id), fetch_type='one')
            return bool(row[0]) if row else False
        except DatabaseError as e:
            self.logger.error(f'Error checking task existence: {e}')
            return False

    # ---------------- Dock Stations ----------------

    def fetch_dock_station(self, dock_type: int) -> DockStation | None:
        """
        Fetch a free dock station of the requested type.

        Free means status = IDLE and no robot holding it. Dock contention is
        not queued: when every dock of a type is busy this returns None and
        the caller simply retries on a later tick.

        Args:
            dock_type: See DockTypeEnum.

        Returns:
            DockStation, or None if no free dock of that type exists.
        """
        try:
            query = """
                SELECT * FROM public.dock_station
                WHERE type = %s
                  AND status = %s
                  AND robot_id IS NULL
                ORDER BY id ASC
                LIMIT 1
            """
            row = self.db.fetch_data(query, params=(dock_type, ChargingStationStatusEnum.IDLE.value), fetch_type='one')

            if row and isinstance(row, tuple):
                return DockStation(*row)

            self.logger.warning(f'No free dock station of type {dock_type} available.')
            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching dock station: {e}')
            return None

    def fetch_free_dock_from(self, dock_ids: list[str], dock_type: int) -> DockStation | None:
        """
        Fetch the first free dock of the requested type from a robot's own list.

        Replaces the global "first free dock of this type" search: a robot may
        only use the docks its namespace lists in config, so one robot can no
        longer take another's charger. Two robots listing the same dock ID
        share it, and this reservation check decides who gets it.

        Args:
            dock_ids: Dock IDs this robot is allowed to use.
            dock_type: See DockTypeEnum.

        Returns:
            DockStation, or None when the robot has no dock of that type free.
        """
        if not dock_ids:
            self.logger.warning(f'Robot has no dock of type {dock_type} configured.')
            return None

        try:
            placeholders = sql.SQL(', ').join(sql.Placeholder() * len(dock_ids))
            query = sql.SQL(
                """
                SELECT * FROM public.dock_station
                WHERE dock_id IN ({ids})
                  AND type = %s
                  AND status = %s
                  AND robot_id IS NULL
                ORDER BY id ASC
                LIMIT 1
                """
            ).format(ids=placeholders)

            params = (*dock_ids, dock_type, ChargingStationStatusEnum.IDLE.value)
            row = self.db.fetch_data(query, params=params, fetch_type='one')

            if row and isinstance(row, tuple):
                return DockStation(*row)

            self.logger.warning(f'No free dock of type {dock_type} among {dock_ids}.')
            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching dock from {dock_ids}: {e}')
            return None

    def fetch_dock_by_type(self, dock_type: int) -> DockStation | None:
        """
        Fetch a dock of the given type, ignoring occupancy.

        Unlike fetch_dock_station this does not require the dock to be free,
        because callers that only need its node - such as costing a return
        trip to the charger - do not care who is currently parked there.

        Args:
            dock_type: See DockTypeEnum.

        Returns:
            DockStation, or None if no dock of that type exists.
        """
        try:
            row = self.db.fetch_data(
                'SELECT * FROM public.dock_station WHERE type = %s ORDER BY id ASC',
                params=(dock_type,),
                fetch_type='one',
            )
            return DockStation(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching dock of type {dock_type}: {e}')
            return None

    def fetch_dock_by_id(self, dock_id: str) -> DockStation | None:
        """
        Fetch a dock station by name, regardless of its occupancy.

        Used when rebuilding an existing dock task, where the dock is already
        reserved by this robot and so would not match fetch_dock_station.

        Args:
            dock_id: Dock name from the dock_station table.

        Returns:
            DockStation, or None if no dock carries that name.
        """
        try:
            row = self.db.fetch_data(
                'SELECT * FROM public.dock_station WHERE dock_id = %s', params=(dock_id,), fetch_type='one'
            )
            return DockStation(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f"Error fetching dock '{dock_id}': {e}")
            return None

    def reserve_dock(self, dock_id: str, robot_id: int) -> bool:
        """
        Reserve a dock for a robot when its docking task is created.

        Args:
            dock_id: Dock name from the dock_station table.
            robot_id: Robot taking the dock.

        Returns:
            True if the reservation was written.
        """
        query = """
            UPDATE public.dock_station
            SET robot_id = %s, status = %s
            WHERE dock_id = %s
        """
        success = self.db.execute_query(query, params=(robot_id, ChargingStationStatusEnum.RESERVED.value, dock_id))
        if success:
            self.logger.info(f"Dock '{dock_id}' reserved for robot {robot_id}")
        else:
            self.logger.error(f"Failed to reserve dock '{dock_id}'")
        return success

    def release_dock_for_robot(self, robot_id: int) -> bool:
        """
        Release whichever dock a robot is holding.

        Called on DONE_UNDOCKING. Keyed on robot rather than dock name because
        the robot's task may already have been cleared by that point.

        Args:
            robot_id: Robot to release docks for.

        Returns:
            True if the update ran (including when the robot held no dock).
        """
        query = """
            UPDATE public.dock_station
            SET robot_id = NULL, status = %s
            WHERE robot_id = %s
        """
        success = self.db.execute_query(query, params=(ChargingStationStatusEnum.IDLE.value, robot_id))
        if not success:
            self.logger.error(f'Failed to release docks held by robot {robot_id}')
        return success

    def fetch_last_dock_node(self, robot_id: int) -> int | None:
        """
        Return the dock node a robot is parked at, or None if it has moved on.

        The robot's most recently created task tells you where it last was. If
        that task was charging or unloading, the robot drove to a dock and has
        not been sent anywhere since, so the dock node is where it still
        stands. Any harvest task after it means the robot left.

        This is task history, not position. The caller checks it against the
        reported pose before trusting it.

        Args:
            robot_id: Database ID of the robot.

        Returns:
            Dock node ID, or None if the latest task was not a dock task.
        """
        try:
            row = self.db.fetch_data(
                """
                SELECT node_id, task_type FROM public.farm_harvesting_job
                WHERE assigned_robot_id = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                params=(robot_id,),
                fetch_type='one',
            )

            if not row:
                return None

            node_id, task_type = int(row[0]), int(row[1])

            if task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
                return node_id

            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching last dock node for robot {robot_id}: {e}')
            return None

    def release_stale_dock_reservations(self) -> int:
        """
        Free docks reserved by robots that have no dock task left to finish.

        A reservation is written when a dock task is created and cleared when
        the robot undocks. Anything that ends the task in between - the server
        restarting, the robot restarting, a task abandoned before docking -
        leaves the row reserved with nothing left to release it, and that dock
        is then unavailable forever.

        Run at start-up: a dock still held by a robot whose dock tasks are all
        closed cannot be genuinely occupied, because the task that reserved it
        no longer exists.

        Returns:
            Number of reservations cleared.
        """
        query = """
            UPDATE public.dock_station d
            SET robot_id = NULL, status = %s
            WHERE d.robot_id IS NOT NULL
              AND NOT EXISTS (
                  SELECT 1 FROM public.farm_harvesting_job j
                  WHERE j.assigned_robot_id = d.robot_id
                    AND j.task_type IN (%s, %s)
                    AND j.progress_status IN (%s, %s, %s)
              )
        """
        rows = self.db.execute_write(
            query,
            params=(
                ChargingStationStatusEnum.IDLE.value,
                TaskEnum.CHARGING_TASK.value,
                TaskEnum.UNLOADING_TASK.value,
                ProgressStatusEnum.IDLE.value,
                ProgressStatusEnum.IN_PROGRESS.value,
                ProgressStatusEnum.PAUSED.value,
            ),
        )

        if rows is None:
            self.logger.error('Failed to sweep stale dock reservations')
            return 0

        if rows:
            self.logger.warning(
                f'Released {rows} stale dock reservation(s) left by a previous run.')

        return rows

    def release_dock(self, dock_id: str) -> bool:
        """
        Release a dock once the robot has undocked.

        Args:
            dock_id: Dock name from the dock_station table.

        Returns:
            True if the release was written.
        """
        query = """
            UPDATE public.dock_station
            SET robot_id = NULL, status = %s
            WHERE dock_id = %s
        """
        success = self.db.execute_query(query, params=(ChargingStationStatusEnum.IDLE.value, dock_id))
        if success:
            self.logger.info(f"Dock '{dock_id}' released")
        else:
            self.logger.error(f"Failed to release dock '{dock_id}'")
        return success

    def update_task_progress(self, task_id: int, job_schedule: str, progress_status: int):
        """
        Update task progress in database.

        Args:
            task_id: ID of the task
            job_schedule: Job schedule identifier
            progress_status: Progress status value (0=not started, 1=in progress,
                             2=paused/error, 3=completed)

        Returns:
            Number of rows affected
        """
        try:
            query = """
                UPDATE public.farm_harvesting_job
                SET progress_status=%s
                WHERE id=%s AND job_schedule=%s
            """
            success = self.db.execute_query(query, params=(progress_status, task_id, job_schedule))

            if success:
                self.logger.debug(
                    f'Updated task progress: task_id={task_id}, schedule={job_schedule}, progress={progress_status}'
                )
            return success
        except DatabaseError as e:
            self.logger.error(f'Error updating task progress: {e}')
            return False

    def mark_job_complete(self, task_id: int, job_schedule: str):
        """
        Mark a task as complete in the database.

        Sets progress_status to COMPLETED, which is what takes the row out of
        fetch_open_task's range. harvest_ready is left alone: it records
        whether the target was ready when the task was created, and is not
        part of the completion check.

        Args:
            task_id: ID of the task
            job_schedule: Job schedule identifier

        Returns:
            bool: True if update succeeded, False otherwise
        """
        try:
            query = """
                UPDATE public.farm_harvesting_job
                SET progress_status = %s
                WHERE id=%s AND job_schedule=%s
            """
            success = self.db.execute_query(query, params=(ProgressStatusEnum.COMPLETED.value, task_id, job_schedule))

            if success:
                self.logger.info(f'Marked job complete: task_id={task_id}, schedule={job_schedule}')
            else:
                self.logger.error(f'Failed to mark job complete: task_id={task_id}, schedule={job_schedule}')
            return success
        except DatabaseError as e:
            self.logger.error(f'Error marking job complete: {e}')
            return False

    # ---------------- Robot Details ------------

    def fetch_robot_details(self, namespace: str) -> tuple | list[tuple] | None:
        """
        Fetch robot details from database by namespace.

        Args:
            namespace (str): Robot namespace to search for.

        Returns:
            list[tuple] | None: List of robot detail tuples or None if not found.
        """
        try:
            result = self.db.fetch_data(
                'SELECT * FROM robot_info WHERE serial_number= %s', params=(namespace,), fetch_type='all'
            )
            self.logger.debug(f'Result: {result}')
            return result
        except Error as e:
            self.logger.error(f'Error fetching robot details: {e}')
            return None

    def mark_robot_offline(self, serial_number: str) -> bool:
        """
        Mark a robot offline in robot_live after the watchdog declares it stale.

        A robot that dies simply stops publishing, so its robot_live row would
        otherwise freeze at its last reported values and the task manager would
        keep assigning work to it. Writing OFFLINE is the only signal the rest
        of the system gets.

        Both columns are written, each from its own enum:
            status      <- RobotStatusEnum.OFFLINE  (97)
            online_flag <- OnlineFlagEnum.OFFLINE   (0)

        No reset is needed on recovery: the robot's next status message
        overwrites both through the normal update path.

        Args:
            serial_number: Robot serial, matching robot_info.serial_number.

        Returns:
            True only if a row was actually updated. A robot with no
            robot_live row yet returns False, so the caller keeps retrying and
            marks it once the row appears, instead of banking a write that
            never happened.
        """
        query = """
            UPDATE public.robot_live
            SET status = %s, online_flag = %s
            WHERE id = (SELECT id FROM public.robot_info WHERE serial_number = %s)
        """
        rows = self.db.execute_write(
            query,
            params=(
                RobotStatusEnum.OFFLINE.value,
                OnlineFlagEnum.OFFLINE.value,
                serial_number,
            ),
        )

        if rows is None:
            self.logger.error(f'Failed to mark robot {serial_number} offline')
            return False

        if rows == 0:
            self.logger.debug(
                f'No robot_live row for {serial_number}; nothing to mark offline')
            return False

        return True

    def fetch_all_robot_details(self) -> list[RobotDetails] | None:
        """
        Fetches the details of all the robots from the database based on the given robot name.

        Args:
            robot_name (str): The serial number of the robot to fetch details for.

        Returns:
            list: A list of rows containing the robot details.

        Raises:
            ProgrammingError: If a programming error occurs while executing the query.
            Exception: If any other unexpected error occurs.
        """
        try:
            result = self.db.fetch_data(query='SELECT * FROM public.robot_info', fetch_type='all')

            if not result:
                raise Error('Robot details not found in the database.')

            return [RobotDetails(*row) for row in result]
        except (Error, TypeError) as e:
            self.logger.error(f'Error fetching robot details: {e}')
            return None

    # ---------------- Farm Edges ----------------

    def fetch_farm_edges(self):
        """Fetch all the farm edges from the database to generate the cost matrix"""
        try:
            result = self.db.fetch_data(query='SELECT * FROM public.farm_edge;', fetch_type='all')

            if not result:
                raise Error('Farm Edge details not found in the database.')

            return [FarmEdges(*row) for row in result]
        except (Error, TypeError) as e:
            self.logger.error(f'Error fetching farm edges: {e}')
            return None

        # ---------------- Farm Nodes ----------------

    def fetch_farm_nodes(self, shortest_route: list[int]) -> list[tuple]:
        """
        Fetches farm node records corresponding to the given route IDs safely using a parameterized query.

        Args:
            shortest_route (list[int]): List of node IDs to fetch.

        Returns:
            list[tuple]: Raw farm node rows, empty if none found or on error.
        """
        if not shortest_route:
            return []

        placeholders = sql.SQL(', ').join(sql.Placeholder() * len(shortest_route))

        farm_node_query = sql.SQL('SELECT * FROM PUBLIC.FARM_NODE WHERE ID IN ({ids})').format(ids=placeholders)

        query_params = tuple(shortest_route)

        result = self.db.fetch_data(query=farm_node_query, params=query_params, fetch_type='all')

        # fetch_data is typed for all three fetch modes; 'all' always yields a
        # list of rows. Normalise so callers get a concrete list.
        return list(result) if isinstance(result, list) else []

    def fetch_farm_assets(self) -> list[FarmAssets]:
        """
        Fetch all harvestable farm assets (bushes), ordered by ID.

        Each asset carries conn_nodes: the approach nodes on either side of
        the bush. One task is generated per (asset, approach node) pair.

        Returns:
            List of FarmAssets, empty if none found or on error.
        """
        if self._assets_cache is not None:
            return self._assets_cache

        try:
            rows = self.db.fetch_data('SELECT * FROM public.farm_asset_on_map ORDER BY id ASC', fetch_type='all')
            self._assets_cache = [FarmAssets(*row) for row in rows] if rows else []
            return self._assets_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching farm assets: {e}')
            return []

    def fetch_node(self, node_id: int) -> FarmNodes | None:
        """
        Fetch a single farm node by ID.

        Note:
            farm_node's primary key is (id, zone_prefix), so an ID could in
            principle appear in more than one zone. The first row is used.

        Args:
            node_id: ID of the node.

        Returns:
            FarmNodes, or None if not found.
        """
        try:
            row = self.db.fetch_data(
                'SELECT * FROM public.farm_node WHERE id = %s ORDER BY zone_prefix ASC',
                params=(node_id,),
                fetch_type='one',
            )
            return FarmNodes(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching node {node_id}: {e}')
            return None

    def fetch_navigable_nodes(self) -> list[FarmNodes]:
        """
        Fetch farm nodes the robot can actually drive to or from.

        A node is navigable only if it appears in farm_edge. Nodes with no
        edges (the 'map' origin marker, for example) sit outside the routing
        graph: the distance matrix is sized from farm_edge, so treating one as
        a robot position makes every path lookup fail and the robot silently
        stops receiving tasks.

        Returns:
            List of FarmNodes with at least one edge, empty on error.
        """
        if self._navigable_nodes_cache is not None:
            return self._navigable_nodes_cache

        try:
            rows = self.db.fetch_data(
                """
                SELECT n.* FROM public.farm_node n
                WHERE EXISTS (
                    SELECT 1 FROM public.farm_edge e
                    WHERE e.from_node = n.id OR e.to_node = n.id
                )
                ORDER BY n.id ASC, n.zone_prefix ASC
                """,
                fetch_type='all'
            )
            self._navigable_nodes_cache = [FarmNodes(*row) for row in rows] if rows else []
            return self._navigable_nodes_cache
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching navigable nodes: {e}')
            return []

    def fetch_all_nodes(self) -> list[FarmNodes]:
        """
        Fetch all farm nodes from database.

        Returns:
            List of FarmNodes objects
        """
        if self._nodes_cache is not None:
            return self._nodes_cache

        try:
            rows = self.db.fetch_data(
                'SELECT * FROM public.farm_node ORDER BY id ASC, zone_prefix ASC', fetch_type='all'
            )
            self._nodes_cache = [FarmNodes(*row) for row in rows] if rows else []
            return self._nodes_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching nodes: {e}')
            return []

    # ---------------- Graph Edges ----------------

    def fetch_graph_edges(self) -> list[GraphEdge] | None:
        """
        Fetch every edge of the topological graph.

        Replaces fetch_farm_edges. graph_edge has no distance column - weight
        is a traversal cost, not metres - so callers that need real distance
        must derive it from the endpoint coordinates instead.

        Returns:
            List of GraphEdge, or None on error.
        """
        try:
            result = self.db.fetch_data(
                query='SELECT source_id, target_id, weight, edge_type FROM public.graph_edge;',
                fetch_type='all',
            )

            if not result:
                raise Error('Graph edge details not found in the database.')

            return [GraphEdge(*row) for row in result]
        except (Error, TypeError) as e:
            self.logger.error(f'Error fetching graph edges: {e}')
            return None

    # ---------------- Graph Nodes ----------------

    def fetch_graph_nodes(self, shortest_route: list[int]) -> list[tuple]:
        """
        Fetch graph node rows for the given route IDs, parameterized.

        Replaces fetch_farm_nodes. Returns raw rows rather than dataclasses
        because TaskGenerator unpacks them positionally when building
        waypoints.

        Args:
            shortest_route: Node IDs to fetch.

        Returns:
            Raw graph_node rows, empty if none found or on error.
        """
        if not shortest_route:
            return []

        placeholders = sql.SQL(', ').join(sql.Placeholder() * len(shortest_route))

        graph_node_query = sql.SQL(
            'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node WHERE id IN ({ids})'
        ).format(ids=placeholders)

        result = self.db.fetch_data(query=graph_node_query, params=tuple(shortest_route), fetch_type='all')

        return list(result) if isinstance(result, list) else []

    def fetch_graph_node(self, node_id: int) -> GraphNode | None:
        """
        Fetch a single graph node by ID.

        Replaces fetch_node. graph_node.id is the whole primary key, so unlike
        farm_node there is no zone to disambiguate and exactly one row can
        match.

        Args:
            node_id: ID of the node.

        Returns:
            GraphNode, or None if not found.
        """
        try:
            row = self.db.fetch_data(
                'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node WHERE id = %s',
                params=(node_id,),
                fetch_type='one',
            )
            return GraphNode(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching graph node {node_id}: {e}')
            return None

    def fetch_all_graph_nodes(self) -> list[GraphNode]:
        """
        Fetch every graph node, ordered by ID.

        Replaces fetch_all_nodes. TaskGenerator needs the full set at start-up
        to build the euclidean distance matrix.

        Returns:
            List of GraphNode, empty on error.
        """
        if self._graph_nodes_cache is not None:
            return self._graph_nodes_cache

        try:
            rows = self.db.fetch_data(
                'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node ORDER BY id ASC',
                fetch_type='all',
            )
            self._graph_nodes_cache = [GraphNode(*row) for row in rows] if rows else []
            return self._graph_nodes_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching graph nodes: {e}')
            return []

    def fetch_navigable_graph_nodes(self) -> list[GraphNode]:
        """
        Fetch graph nodes the robot can actually drive to or from.

        Replaces fetch_navigable_nodes. Every node in the current graph has at
        least one edge, so the EXISTS clause filters nothing today. It is kept
        because the cost matrix is sized from graph_edge: an edgeless node
        added later would make every path lookup from it fail, and the robot
        would silently stop receiving tasks.

        Returns:
            List of GraphNode with at least one edge, empty on error.
        """
        if self._navigable_graph_nodes_cache is not None:
            return self._navigable_graph_nodes_cache

        try:
            rows = self.db.fetch_data(
                """
                SELECT n.id, n.node_type, n.x, n.y, n.theta, n.obj_id
                FROM public.graph_node n
                WHERE EXISTS (
                    SELECT 1 FROM public.graph_edge e
                    WHERE e.source_id = n.id OR e.target_id = n.id
                )
                ORDER BY n.id ASC
                """,
                fetch_type='all',
            )
            self._navigable_graph_nodes_cache = [GraphNode(*row) for row in rows] if rows else []
            return self._navigable_graph_nodes_cache
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching navigable graph nodes: {e}')
            return []

    # ---------------- Harvest Units ----------------

    def fetch_harvest_units(self) -> list[HarvestUnit]:
        """
        Fetch every side of every bush, already in sweep order.

        Replaces fetch_farm_assets. Each object is served by two Pickup nodes,
        one per side of the row, and each side is a separate unit of work
        because the manipulator reaches only half the bush from either side.

        The sweep is ordered row by row, and within a row one lane at a time:

            row 1 near lane, node IDs ascending   -> b1_1 .. b1_38
            row 1 far lane,  node IDs ascending   -> b1_38 .. b1_1
            row 2 near lane, ...

        Node IDs run opposite to object IDs on the far lane, so ordering both
        lanes by node ID drives down one side of the row and back up the other
        as a single loop, and finishes each row before starting the next.

        Objects that do not have exactly two Pickup nodes are dropped rather
        than half-harvested; the count is logged so a broken map is visible.

        Returns:
            Ordered list of HarvestUnit, empty if none found or on error.
        """
        if self._harvest_units_cache is not None:
            return self._harvest_units_cache

        try:
            rows = self.db.fetch_data(
                """
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
                """,
                fetch_type='all',
            )

            if not rows:
                self.logger.error('No harvest units found: object_data or graph_node is empty.')
                self._harvest_units_cache = []
                return self._harvest_units_cache

            units = [
                HarvestUnit(
                    name=f'b{row_id}_{object_id}',
                    node_id=node_id,
                    row_id=row_id,
                    x=float(x_coord),
                    y=float(y_coord),
                )
                for row_id, object_id, node_id, x_coord, y_coord, _lane in rows
            ]

            expected = self.count_objects()
            if expected is not None and len(units) != expected * 2:
                self.logger.warning(
                    f'Harvest sweep has {len(units)} units but object_data holds {expected} objects; '
                    f'{expected * 2 - len(units)} sides dropped for not having exactly two Pickup nodes.'
                )

            self._harvest_units_cache = units
            return units
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching harvest units: {e}')
            return []

    def fetch_object_positions(self) -> list[tuple[float, float]]:
        """
        Fetch every object's map position.

        Used to push published waypoints off the crop: the graph places nodes
        where the manipulator wants them, which is inside the band Nav2 treats
        as untraversable around a mapped obstacle.

        Returns:
            (x, y) per object, empty on error or when the schema has none.
        """
        if self._object_positions_cache is not None:
            return self._object_positions_cache

        if not self._use_graph_tables:
            # farm_asset_on_map is the legacy equivalent; keeping the shift
            # available on rollback avoids a behaviour change between sources.
            assets = self.fetch_farm_assets()
            self._object_positions_cache = [(float(a.x_map), float(a.y_map)) for a in assets]
            return self._object_positions_cache

        try:
            rows = self.db.fetch_data('SELECT x_coord, y_coord FROM public.object_data', fetch_type='all')
            self._object_positions_cache = [(float(r[0]), float(r[1])) for r in rows] if rows else []
            return self._object_positions_cache
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching object positions: {e}')
            return []

    def count_objects(self) -> int | None:
        """
        Count rows in object_data, used to detect bushes dropped from the sweep.

        Returns:
            Row count, or None on error.
        """
        try:
            row = self.db.fetch_data('SELECT COUNT(*) FROM public.object_data', fetch_type='one')
            return int(row[0]) if row and isinstance(row, tuple) else None
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error counting objects: {e}')
            return None

    # ---------------- Map Dispatch ----------------
    #
    # Everything that routes calls these rather than the table-specific
    # readers above, so `farm.graph_source` is the only place that knows which
    # schema is live. Both families return the same shapes: normalised edge
    # triples, node objects exposing id / x / y / theta, and HarvestUnits.

    def fetch_route_edges(self) -> list[tuple[int, int, float]] | None:
        """
        Fetch the routing graph as (source, target, cost) triples.

        The cost is graph_edge.weight, or farm_edge.distance on the legacy
        schema. Neither is guaranteed to be metres - weight is a traversal
        cost - so callers needing real distance derive it from node
        coordinates instead.

        Returns:
            Edge triples, or None on error.
        """
        if self._use_graph_tables:
            edges = self.fetch_graph_edges()
            return [(e.source_id, e.target_id, e.weight) for e in edges] if edges else None

        farm_edges = self.fetch_farm_edges()
        return [(e.from_node, e.to_node, e.distance) for e in farm_edges] if farm_edges else None

    def fetch_route_nodes(self, node_ids: list[int]) -> list[GraphNode] | list[FarmNodes]:
        """
        Fetch the nodes on a route, as objects rather than raw rows.

        Args:
            node_ids: Node IDs to fetch.

        Returns:
            Node objects exposing id, x, y and theta. Empty on error.
        """
        rows = self.fetch_graph_nodes(node_ids) if self._use_graph_tables else self.fetch_farm_nodes(node_ids)

        if not rows:
            return []

        node_class = GraphNode if self._use_graph_tables else FarmNodes
        return [node_class(*row) for row in rows]

    def fetch_map_node(self, node_id: int) -> GraphNode | FarmNodes | None:
        """
        Fetch one node by ID from whichever schema is live.

        Args:
            node_id: ID of the node.
        """
        return self.fetch_graph_node(node_id) if self._use_graph_tables else self.fetch_node(node_id)

    def fetch_all_map_nodes(self) -> list[GraphNode] | list[FarmNodes]:
        """Fetch every node from whichever schema is live."""
        return self.fetch_all_graph_nodes() if self._use_graph_tables else self.fetch_all_nodes()

    def fetch_navigable_map_nodes(self) -> list[GraphNode] | list[FarmNodes]:
        """Fetch every node with at least one edge, from whichever schema is live."""
        return self.fetch_navigable_graph_nodes() if self._use_graph_tables else self.fetch_navigable_nodes()

    def fetch_sweep(self) -> list[HarvestUnit]:
        """
        Fetch every side of every bush, in the order they should be harvested.

        On the graph schema the ordering is done in SQL, row by row. On the
        legacy schema it is rebuilt here from farm_asset_on_map so the old
        behaviour survives a rollback: one pass over every bush ascending
        taking the lower conn_node, then one descending taking the higher.
        That sweep crosses the whole farm twice instead of finishing a row at
        a time, which is the behaviour the graph schema replaces.

        Returns:
            Ordered HarvestUnits, empty if none found or on error.
        """
        if self._use_graph_tables:
            return self.fetch_harvest_units()

        assets = self.fetch_farm_assets()
        if not assets:
            self.logger.error('No farm assets found in the database.')
            return []

        sweep: list[HarvestUnit] = []

        for asset in assets:
            if asset.conn_nodes:
                sweep.append(
                    HarvestUnit(
                        name=asset.name,
                        node_id=min(asset.conn_nodes),
                        row_id=0,
                        x=float(asset.x_map),
                        y=float(asset.y_map),
                    )
                )

        for asset in reversed(assets):
            if len(asset.conn_nodes or []) > 1:
                sweep.append(
                    HarvestUnit(
                        name=asset.name,
                        node_id=max(asset.conn_nodes),
                        row_id=0,
                        x=float(asset.x_map),
                        y=float(asset.y_map),
                    )
                )

        return sweep

    # ---------------- Cleanup ----------------

    def close(self):
        """Close the database connection"""
        self.db.close()
        self.logger.info('Database connection closed')

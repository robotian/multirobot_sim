# Base station: stands in for the farm's base-station PC. A ROS 2 Jazzy computer on the robots' network
# (simulated or real) that also hosts the harvesting operation's PostgreSQL database. Built on the robot image
# so the ROS packages, RMWs and Python deps (psycopg for status_server) match the robots' exactly; build that
# first (`docker compose build robot0`).
ARG BASE_IMAGE=clearpath-robot:jazzy
FROM ${BASE_IMAGE}

# PostgreSQL from the PGDG repo rather than Ubuntu's 16, so pg_restore can read dumps made by a newer server
# (it refuses archives from a newer pg_dump). A data volume is tied to its major version: changing PG_MAJOR
# needs a dump, `docker volume rm basestation_pgdata` and a restore.
ARG PG_MAJOR=18
ENV PG_MAJOR=${PG_MAJOR}
RUN install -d /usr/share/postgresql-common/pgdg \
    && curl -fsSL -o /usr/share/postgresql-common/pgdg/apt.postgresql.org.asc https://www.postgresql.org/media/keys/ACCC4CF8.asc \
    && echo "deb [signed-by=/usr/share/postgresql-common/pgdg/apt.postgresql.org.asc] https://apt.postgresql.org/pub/repos/apt noble-pgdg main" > /etc/apt/sources.list.d/pgdg.list \
    # no default cluster at install: entrypoint.sh creates its own in the data volume
    && mkdir -p /etc/postgresql-common/createcluster.d \
    && echo 'create_main_cluster = false' > /etc/postgresql-common/createcluster.d/no-main.conf \
    && apt-get update && apt-get install -y --no-install-recommends postgresql-${PG_MAJOR} \
    && rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/lib/postgresql/${PG_MAJOR}/bin:$PATH \
    PGDATA=/var/lib/postgresql/data \
    PGHOST=/run/postgresql

# The robot image's /etc/ros_env.sh prefers /etc/clearpath/setup.bash, which only a robot's entrypoint
# writes; here it's plain ROS plus the shared colcon_ws (RMW/domain come from the environment).
RUN printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        '[ -f /home/robot/colcon_ws/install/setup.bash ] && source /home/robot/colcon_ws/install/setup.bash' \
        'true' > /etc/ros_env.sh

COPY entrypoint.sh /basestation-entrypoint.sh
RUN chmod +x /basestation-entrypoint.sh
ENTRYPOINT ["/basestation-entrypoint.sh"]
CMD ["sleep", "infinity"]

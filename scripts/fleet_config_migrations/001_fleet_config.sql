-- The fleet's settings (scripts/fleetcfg.py, the web UI's Configuration page) in fleet_config.sqlite, this
-- checkout's own SQLite file (gitignored): every PC has one, no server needed. What each setting means (type,
-- default, which containers it reaches) is scripts/fleet_settings.py; the database holds the values. .env is
-- generated from the active profile. Same tables as the PostgreSQL fleet_config the base station held before
-- (2026-10-09/10), which the first use imports from when it can reach it.
-- Times are UTC ISO 8601 text; booleans 0/1; JSON as text. The change log is written by fleetcfg.py (not a
-- trigger), so edits made with another SQLite client are not logged.

-- A named set of settings and robot slots; exactly one is active (active_profile).
CREATE TABLE IF NOT EXISTS profile (
    id integer PRIMARY KEY AUTOINCREMENT,
    name text NOT NULL UNIQUE CHECK (length(name) BETWEEN 1 AND 63 AND name NOT GLOB '*[^A-Za-z0-9 ._-]*'
                                     AND name GLOB '[A-Za-z0-9]*'),
    notes text NOT NULL DEFAULT '',
    created_at text NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS active_profile (
    only_row integer PRIMARY KEY DEFAULT 1 CHECK (only_row = 1),
    profile_id integer NOT NULL REFERENCES profile (id)
);

-- One row per setting the profile sets (an .env variable); a setting without a row takes its default.
CREATE TABLE IF NOT EXISTS setting_value (
    profile_id integer NOT NULL REFERENCES profile (id) ON DELETE CASCADE,
    key text NOT NULL CHECK (key GLOB '[A-Z]*' AND key NOT GLOB '*[^A-Z0-9_]*'),
    value text NOT NULL,
    PRIMARY KEY (profile_id, key)
);

-- Robot slots 0-7: model and spawn pose (.env ROBOT_MODEL_<i> / ROBOT_POSE_<i>). Slots >= NUM_ROBOTS are kept
-- (remembered for the next time the fleet grows) but not started. A NULL pose = the sim's default layout.
CREATE TABLE IF NOT EXISTS robot_slot (
    profile_id integer NOT NULL REFERENCES profile (id) ON DELETE CASCADE,
    slot integer NOT NULL CHECK (slot BETWEEN 0 AND 7),
    model text NOT NULL CHECK (model GLOB '[a-z0-9]*' AND model NOT GLOB '*[^a-z0-9_]*'),
    x real,
    y real,
    yaw_deg real,
    PRIMARY KEY (profile_id, slot),
    CHECK ((x IS NULL) = (y IS NULL) AND (x IS NULL) = (yaw_deg IS NULL))
);

-- MTU's real robots reached over SSH (the web UI's Real robots card). Not per profile.
CREATE TABLE IF NOT EXISTS real_robot (
    id text PRIMARY KEY CHECK (id GLOB '[a-z0-9]*_[0-9]*' AND id NOT GLOB '*[^a-z0-9_]*'),
    host text NOT NULL,
    ssh_user text NOT NULL DEFAULT 'robot',
    cutter integer NOT NULL DEFAULT 0 CHECK (cutter IN (0, 1))
);

-- Every change fleetcfg.py makes to the tables above, with who made it (user@host, or "web UI").
CREATE TABLE IF NOT EXISTS change_log (
    id integer PRIMARY KEY AUTOINCREMENT,
    at text NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
    actor text NOT NULL,
    tbl text NOT NULL,
    op text NOT NULL,
    old_row text,
    new_row text
);
CREATE INDEX IF NOT EXISTS change_log_at ON change_log (at DESC);

-- Each start of the sim or spawn through scripts/fleet.sh: what it ran with and how it ended.
CREATE TABLE IF NOT EXISTS run (
    id integer PRIMARY KEY AUTOINCREMENT,
    started_at text NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now')),
    finished_at text,
    actor text NOT NULL,
    command text NOT NULL,
    profile text,
    git_commit text,
    env text NOT NULL,
    exit_code integer
);
CREATE INDEX IF NOT EXISTS run_started_at ON run (started_at DESC);

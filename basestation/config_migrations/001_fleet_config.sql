-- The fleet's settings (scripts/fleetcfg.py, the web UI's Configuration page), in the base station's database
-- fleet_config: this PC's own, separate from the farm database, so scripts/db_sync.sh never copies or replaces it.
-- What each setting means (type, default, which containers it reaches) is scripts/fleet_settings.py, in git with
-- the code that reads it; the database holds the values. .env is generated from the active profile.

-- A named set of settings and robot slots; exactly one is active (active_profile).
CREATE TABLE IF NOT EXISTS profile (
    id serial PRIMARY KEY,
    name text NOT NULL UNIQUE CHECK (name ~ '^[A-Za-z0-9][A-Za-z0-9 ._-]{0,62}$'),
    notes text NOT NULL DEFAULT '',
    created_at timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS active_profile (
    only_row boolean PRIMARY KEY DEFAULT true CHECK (only_row),
    profile_id integer NOT NULL REFERENCES profile (id)
);

-- One row per setting the profile sets (an .env variable); a setting without a row takes its default.
CREATE TABLE IF NOT EXISTS setting_value (
    profile_id integer NOT NULL REFERENCES profile (id) ON DELETE CASCADE,
    key text NOT NULL CHECK (key ~ '^[A-Z][A-Z0-9_]*$'),
    value text NOT NULL,
    PRIMARY KEY (profile_id, key)
);

-- Robot slots 0-7: model and spawn pose (.env ROBOT_MODEL_<i> / ROBOT_POSE_<i>). Slots >= NUM_ROBOTS are kept
-- (remembered for the next time the fleet grows) but not started. A NULL pose = the sim's default layout.
CREATE TABLE IF NOT EXISTS robot_slot (
    profile_id integer NOT NULL REFERENCES profile (id) ON DELETE CASCADE,
    slot integer NOT NULL CHECK (slot BETWEEN 0 AND 7),
    model text NOT NULL CHECK (model ~ '^[a-z0-9]+(_[0-9]+)?$'),
    x double precision,
    y double precision,
    yaw_deg double precision,
    PRIMARY KEY (profile_id, slot),
    CHECK ((x IS NULL) = (y IS NULL) AND (x IS NULL) = (yaw_deg IS NULL))
);

-- MTU's real robots reached over SSH (the web UI's Real robots card; was tools/sim_ui/real_robots.json).
-- Not per profile: a robot's address doesn't change with what is being simulated.
CREATE TABLE IF NOT EXISTS real_robot (
    id text PRIMARY KEY CHECK (id ~ '^[a-z0-9]+_[0-9]+$'),
    host text NOT NULL,
    ssh_user text NOT NULL DEFAULT 'robot',
    cutter boolean NOT NULL DEFAULT false
);

-- Every change to the tables above (trigger log_change), with who made it: the session's fleet.actor setting
-- (scripts/fleetcfg.py sets it to user@host or "web UI"), else the database user. A transaction with fleet.bulk
-- 'on' (creating, copying or deleting a whole profile) logs only the profile, not each of its settings and slots.
CREATE TABLE IF NOT EXISTS change_log (
    id bigserial PRIMARY KEY,
    at timestamptz NOT NULL DEFAULT now(),
    actor text NOT NULL,
    tbl text NOT NULL,
    op text NOT NULL,
    old_row jsonb,
    new_row jsonb
);
CREATE INDEX IF NOT EXISTS change_log_at ON change_log (at DESC);

-- Each start of the sim or spawn through scripts/fleet.sh: what it ran with and how it ended.
CREATE TABLE IF NOT EXISTS run (
    id bigserial PRIMARY KEY,
    started_at timestamptz NOT NULL DEFAULT now(),
    finished_at timestamptz,
    actor text NOT NULL,
    command text NOT NULL,
    profile text,
    git_commit text,
    env jsonb NOT NULL,
    exit_code integer
);
CREATE INDEX IF NOT EXISTS run_started_at ON run (started_at DESC);

CREATE OR REPLACE FUNCTION log_change() RETURNS trigger LANGUAGE plpgsql AS $$
BEGIN
    IF TG_OP = 'UPDATE' AND OLD IS NOT DISTINCT FROM NEW THEN
        RETURN NULL;
    END IF;
    IF current_setting('fleet.bulk', true) = 'on' AND TG_TABLE_NAME IN ('setting_value', 'robot_slot') THEN
        RETURN NULL;
    END IF;
    INSERT INTO change_log (actor, tbl, op, old_row, new_row)
    VALUES (coalesce(nullif(current_setting('fleet.actor', true), ''), session_user), TG_TABLE_NAME, TG_OP,
            CASE WHEN TG_OP <> 'INSERT' THEN to_jsonb(OLD) END,
            CASE WHEN TG_OP <> 'DELETE' THEN to_jsonb(NEW) END);
    RETURN NULL;
END $$;

DO $$
DECLARE t text;
BEGIN
    FOREACH t IN ARRAY ARRAY['profile', 'active_profile', 'setting_value', 'robot_slot', 'real_robot'] LOOP
        IF NOT EXISTS (SELECT 1 FROM pg_trigger WHERE tgname = t || '_log_change') THEN
            EXECUTE format('CREATE TRIGGER %I AFTER INSERT OR UPDATE OR DELETE ON %I '
                           'FOR EACH ROW EXECUTE FUNCTION log_change()', t || '_log_change', t);
        END IF;
    END LOOP;
END $$;

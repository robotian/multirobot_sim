-- Charging stations, read by scripts/make_charger_scene.py (sim/scripts/add_chargers.py) to place them in the sim.
-- Same definition as the table first made by hand on another PC (pg_dump --schema-only there, 2026-10-09): on a
-- database that has it, CREATE is skipped and the comments are set to the same text. Rows are data, not schema:
-- copy them with scripts/db_sync.sh.
CREATE TABLE IF NOT EXISTS public.charging_stations (
    id integer NOT NULL,
    charger_model text,
    x_coord double precision,
    y_coord double precision,
    yaw_coord_deg double precision,
    apriltag_id integer,
    apriltag_sz_mm integer,
    frame text,
    CONSTRAINT charging_stations_pkey PRIMARY KEY (id),
    CONSTRAINT charging_stations_apriltag_id_key UNIQUE (apriltag_id)
);

COMMENT ON COLUMN public.charging_stations.charger_model IS 'Model, e.g. TR-302 (WiBotic TR-302 Edge); sim asset per model in sim/scripts/add_chargers.py';
COMMENT ON COLUMN public.charging_stations.x_coord IS 'Footprint centre of the charger on the ground, m, in frame';
COMMENT ON COLUMN public.charging_stations.y_coord IS 'Footprint centre of the charger on the ground, m, in frame';
COMMENT ON COLUMN public.charging_stations.yaw_coord_deg IS 'Direction the charger''s front (AprilTag side) faces, degrees, CCW from the +X axis of frame';
COMMENT ON COLUMN public.charging_stations.apriltag_sz_mm IS 'Edge of the tag''s black square, mm (apriltag_ros size)';
COMMENT ON COLUMN public.charging_stations.frame IS 'Frame of x_coord / y_coord / yaw_coord_deg, e.g. map';

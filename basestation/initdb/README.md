Files here are loaded, in name order, when the base station creates its database (first start, empty
`basestation_pgdata` volume): `*.sql` and `*.sql.gz` with `psql`, `*.dump` (`pg_dump -Fc`) with `pg_restore
--no-owner`, `*.sh` with bash. All run as the database superuser (`admin`) against the database
(`test_lavender_farming`). Everything here except this README is gitignored.

From an existing farm database, e.g. the old standalone `robotian_database` container (port 5432):

    docker exec robotian_database pg_dump -U admin -Fc test_lavender_farming > basestation/initdb/10-farm.dump

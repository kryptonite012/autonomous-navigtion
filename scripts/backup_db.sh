#!/usr/bin/env bash
# Back up the NEXUS database. SQLite uses the online backup API; PostgreSQL uses pg_dump.
set -euo pipefail
cd "$(dirname "$0")/.."
[[ -f .env ]] && set -a && . ./.env && set +a
URL=${NEXUS_DATABASE_URL:-sqlite:///data/nexus.db}
mkdir -p backups
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
if [[ $URL == sqlite* ]]; then
  PY=${PYTHON:-.venv/bin/python}; [[ -x $PY ]] || PY=python3
  $PY -c "from backend.database.db import Database; print(Database().backup())"
else
  PGURL=${URL/postgresql+psycopg/postgresql}
  pg_dump "$PGURL" | gzip > "backups/nexus-$STAMP.sql.gz"
  echo "backups/nexus-$STAMP.sql.gz"
fi
# keep the 14 most recent backups
ls -1t backups/nexus-* 2>/dev/null | tail -n +15 | xargs -r rm --

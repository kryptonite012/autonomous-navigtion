# Database

SQLAlchemy 2 models in `backend/database/models.py`. SQLite by default
(`data/nexus.db`, WAL mode); PostgreSQL works by setting
`NEXUS_DATABASE_URL=postgresql+psycopg://user:pass@host/nexus` and
installing `psycopg[binary]`.

## ER diagram

```mermaid
erDiagram
    USERS ||--o{ AUTH_SESSIONS : has
    USERS {
        int id PK
        string username UK
        string password_hash
        string role "viewer|operator|admin"
        bool is_active
        int failed_logins
        datetime locked_until
        datetime created_at
    }
    AUTH_SESSIONS {
        string id PK "token sid"
        int user_id FK
        datetime created_at
        datetime expires_at
        bool revoked
        string client_ip
    }
    ROBOT_SESSIONS {
        int id PK
        datetime start_time
        datetime end_time
        string operating_mode
        string session_status
        string hardware_mode
    }
    MAPS {
        int id PK
        string map_name UK
        string file_path
        float resolution
        bool experimental
        datetime created_at
    }
    OBJECT_DETECTIONS {
        int id PK
        int object_id
        string color
        float confidence
        float centroid_x
        float centroid_y
        float area
        datetime timestamp
    }
    SORTING_RECORDS {
        int id PK
        int object_id
        string detected_color
        int target_bin
        string sorting_status
        int attempts
        text message
        datetime timestamp
    }
    TELEMETRY {
        int id PK
        float battery_voltage
        float linear_velocity
        float angular_velocity
        float cpu_temperature
        float pose_x
        float pose_y
        float pose_theta
        datetime timestamp
    }
    SYSTEM_EVENTS {
        int id PK
        string event_type
        string severity
        text description
        datetime timestamp
    }
    AUDIT_LOG {
        int id PK
        string username
        string action
        text detail
        string client_ip
        bool success
        datetime timestamp
    }
    MISSIONS {
        int id PK
        string name UK
        text definition "JSON waypoints, loop, policy"
        datetime created_at
        datetime updated_at
    }
    SCHEMA_VERSION {
        int version PK
        datetime applied_at
        string description
    }
```

## What is stored and how often

| Table | Written when | Volume control |
|---|---|---|
| `object_detections` | first sighting of each tracked object (not every frame) | pruned after 14 days |
| `telemetry` | every 5 s | pruned after 14 days |
| `sorting_records` | every completed sort attempt | kept |
| `system_events` | every runtime event | kept |
| `audit_log` | every authenticated mutating action and login | kept |
| `missions` | when a mission is saved/deleted (migration 3) | kept |
| camera frames | **never** — only the latest JPEG is held in memory | — |

`sorting_status` values: `success` (bin sensor confirmed), `unverified`
(no bin sensor), `failed_pickup`, `failed_verification`.

## Migrations

`Database.migrate()` runs at startup. Each migration in
`backend/database/db.py::MIGRATIONS` is applied once and recorded in
`schema_version`. To change the schema, append a new numbered migration;
never edit an applied one.

## Backups

* Dashboard: *Settings → Back up database* (admin).
* CLI: `scripts/backup_db.sh` (SQLite online backup API, or `pg_dump`);
  keeps the 14 newest files in `backups/`.
* Automatic: `nexus-backup.timer` runs it daily.
* Restore (service stopped):
  `python3 -c "from backend.database.db import Database; Database().restore('backups/nexus-XXXX.db')"`

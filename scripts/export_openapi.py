#!/usr/bin/env python3
"""Write docs/openapi.json and the endpoint table in docs/api.md from the live app."""

import json
import os
import secrets
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from backend.main import create_app  # noqa: E402

ROLE_HINT = {"viewer": "viewer", "operator": "operator", "admin": "admin"}


def main() -> int:
    # throwaway in-memory database: don't let the admin bootstrap write a
    # password file over the real one in data/
    os.environ["NEXUS_ADMIN_PASSWORD"] = secrets.token_urlsafe(16) + "Aa1!"
    app = create_app(db_url="sqlite://", secret="x" * 40, start_robot=False)
    spec = app.openapi()
    (ROOT / "docs" / "openapi.json").write_text(json.dumps(spec, indent=1))
    rows = []
    for path, ops in sorted(spec["paths"].items()):
        for method, op in ops.items():
            rows.append(f"| `{method.upper()}` | `{path}` | {op.get('summary', '')} | "
                        f"{(op.get('description') or '').splitlines()[0] if op.get('description') else ''} |")
    table = "| Method | Path | Summary | Notes |\n|---|---|---|---|\n" + "\n".join(rows)
    doc = ROOT / "docs" / "api.md"
    text = doc.read_text()
    start, end = "<!-- endpoints:start -->", "<!-- endpoints:end -->"
    text = text[:text.index(start) + len(start)] + "\n" + table + "\n" + text[text.index(end):]
    doc.write_text(text)
    print(f"{len(rows)} endpoints written")
    return 0


if __name__ == "__main__":
    sys.exit(main())

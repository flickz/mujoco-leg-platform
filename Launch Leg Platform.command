#!/bin/bash
set -euo pipefail
pkg="$(cd -- "$(dirname -- "$0")" && pwd)"
py="$pkg/.venv/bin/python"
if [[ ! -x "$py" ]]; then
  echo "Create the Python environment first: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
  exit 1
fi
exec "$py" "$pkg/.venv/bin/mjpython" "$pkg/viewer.py" "$@"

#!/bin/bash
set -euo pipefail
pkg="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "$pkg/Launch Leg Platform.command" --free

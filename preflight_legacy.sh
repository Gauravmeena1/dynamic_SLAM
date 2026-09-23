#!/usr/bin/env bash
# Legacy compatibility alias for existing field notes and operator habits.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "提示 / Note: legacy preflight alias; use preflight.sh for new workflows." >&2
exec "$HERE/preflight.sh" "$@"

#!/bin/bash
# One-command SLAM evaluation suite.
# Usage: ./run_suite.sh [user_config.yaml]
# See user_config.yaml for configuration.
set -e
SUITE_DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$SUITE_DIR/scripts/run_suite.py" "$@"

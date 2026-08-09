#!/usr/bin/env bash
# Run pure-Python navigation tests (no ROS / Gazebo required).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
python3 tests/test_navigation_geometry.py -v

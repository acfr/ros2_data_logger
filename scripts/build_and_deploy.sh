#!/bin/bash
# Quick build and deploy script
# Usage: ./scripts/build_and_deploy.sh [--ros-distro humble|iron|jazzy]

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments and forward to build script
"${SCRIPT_DIR}/build_deb.sh" "$@"

# Forward to deploy script
"${SCRIPT_DIR}/deploy.sh" "$@"

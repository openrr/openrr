#!/bin/bash

# Update schemas for configs.
#
# Usage:
#    ./tools/update-schema.sh

set -euo pipefail
IFS=$'\n\t'

cd "$(cd "$(dirname "${0}")" && pwd)"/..

cargo run -p openrr-apps --bin openrr_apps_config -- schema robot-config >openrr-apps/schema/robot_config.json
cargo run -p openrr-apps --bin openrr_apps_config -- schema robot-teleop-config >openrr-apps/schema/robot_teleop_config.json

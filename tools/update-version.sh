#!/bin/bash

# Update version in Cargo.toml.
#
# Usage:
#    ./tools/update-version.sh [options]
#    ./tools/update-version.sh
#
# Note: This script requires cargo-workspaces <https://github.com/pksunkara/cargo-workspaces>

set -euo pipefail
IFS=$'\n\t'

cargo workspaces version --force '*' --no-git-commit patch -y "$@"

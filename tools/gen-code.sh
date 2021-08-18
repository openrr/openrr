#!/bin/bash

# Run code generators.
#
# Usage:
#    ./tools/gen-code.sh

set -euo pipefail
IFS=$'\n\t'

cd "$(cd "$(dirname "$0")" && pwd)"/..

cargo run --manifest-path tools/codegen/Cargo.toml

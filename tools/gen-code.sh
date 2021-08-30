#!/bin/bash

# Run code generators.
#
# Usage:
#    ./tools/gen-code.sh

set -euo pipefail
IFS=$'\n\t'

cd "$(cd "$(dirname "$0")" && pwd)"/..

# generate openrr-remote/src/generated/*.rs for codegen
cargo build --manifest-path openrr-remote/Cargo.toml

cargo run --manifest-path tools/codegen/Cargo.toml

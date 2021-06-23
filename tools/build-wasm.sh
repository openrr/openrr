#!/bin/bash

# Build openrr for wasm32-unknown-unknown.
#
# Usage:
#    ./tools/build-wasm.sh

set -euo pipefail
IFS=$'\n\t'

# arci-urdf-viz and its dependencies support wasm.
cargo build --target wasm32-unknown-unknown -p arci-urdf-viz

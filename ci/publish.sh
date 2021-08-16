#!/bin/bash

set -euo pipefail
IFS=$'\n\t'

# A list of paths to the crate to be published.
# It will be published in the order listed.
MEMBERS=(
  "arci"
  "openrr-sleep"
  "openrr-planner"
  "openrr-config"

  # depend on arci
  "openrr-plugin"
  "openrr-test"

  # depend on arci and openrr-planner
  "openrr-client"

  # depend on arci and openrr-client
  "openrr-command"
  "openrr-gui"

  # depend on arci, openrr-client and openrr-command
  "openrr-teleop"

  # depend on arci and some openrr-* crates
  "arci-gamepad-gilrs"
  "arci-gamepad-keyboard"
  "arci-ros"
  "arci-speak-audio"
  "arci-speak-cmd"
  "arci-urdf-viz"

  # depend on all arci-* crates
  "openrr-apps"

  # depend on all openrr-* crates
  "openrr"
)

cd "$(cd "$(dirname "${0}")" && pwd)"/..

set -x

for i in "${!MEMBERS[@]}"; do
  (
    cd "${MEMBERS[${i}]}"
    cargo clean
    cargo publish
  )
  if [[ $((i + 1)) != "${#MEMBERS[@]}" ]]; then
    sleep 45
  fi
done

#!/bin/bash

set -euo pipefail
IFS=$'\n\t'

# A list of paths to the crate to be published.
# It will be published in the order listed.
MEMBERS=(
    "arci"
    "openrr-sleep"
    "openrr-planner"

    # depend on arci and openrr-planner
    "openrr-client"

    # depend on arci and openrr-client
    "openrr-command"
    "openrr-gui"
    "openrr-teleop"

    # depend on arci and some openrr-* crates
    "arci-gamepad-gilrs"
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
        cargo publish
    )
    if [[ $((i + 1)) != "${#MEMBERS[@]}" ]]; then
        sleep 30
    fi
done

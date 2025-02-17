#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [[ -f /proc/sys/kernel/core_pattern ]]; then
    mkdir -p /tmp/core_dumps
    find . -name "core.*" -exec cp \{\} /tmp/core_dumps \;
fi

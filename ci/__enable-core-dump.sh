#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [[ -f /proc/sys/kernel/core_pattern ]]; then
    ulimit -c unlimited
fi

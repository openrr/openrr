#!/bin/bash

set -euo pipefail
IFS=$'\n\t'

sudo apt-get update

# libudev-dev is for arci-gamepad-gilrs
# libasound2-dev is for arci-speak-audio
sudo apt-get install -y \
    libudev-dev libasound2-dev

#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/kilted/setup.bash" --
exec "$@"

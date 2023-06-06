#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
exec "$@"

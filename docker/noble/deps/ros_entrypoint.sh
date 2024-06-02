#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash" --
exec "$@"

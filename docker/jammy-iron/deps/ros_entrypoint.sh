#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/iron/setup.bash" --
exec "$@"

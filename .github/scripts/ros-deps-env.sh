# Puts the graph optimizers taken from the ROS 2 repo (see cmake-linux.yml) on
# the loader path, for a shell that is about to run something built against
# them. Sourced, not executed: it exports into the caller.
#
#   source .github/scripts/ros-deps-env.sh /opt/ros/humble
#
# A missing or empty prefix is a build that took every dependency from the
# Ubuntu archive: nothing to add.
#
# Both directories are needed. GTSAM and g2o install their libraries in the
# multiarch subdirectory, while the prefix's own lib/ holds the rest, and
# libgtsam.so carries no RUNPATH -- so the loader finds neither it nor the
# libmetis-gtsam.so it pulls in without being told where to look.
ros_prefix="${1:-}"
if [ -n "$ros_prefix" ] && [ -d "$ros_prefix" ]; then
  ros_lib="$ros_prefix/lib"
  export LD_LIBRARY_PATH="${ros_lib}:${ros_lib}/$(gcc -dumpmachine)${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
fi

#!/bin/bash

SKIP=0
if [ $# -eq 1 ]
then
  SKIP=$1
fi

DETECTOR=(0 1 6 7 9 11 12 14) #0 1 6 7 8 9 11 12 13 14

PREFIX="/home/mathieu/workspace/rtabmap_cv_latest/bin/"
REPORT_TOOL="${PREFIX}rtabmap-report"

for d in "${DETECTOR[@]}"
do
  $REPORT_TOOL --export --export_prefix "Stat$d" --loc 32  Loop/Odom_correction_norm/m Loop/Visual_inliers/ Timing/Total/ms Loop/Map_id/ Keypoint/Current_frame/words Memory/RAM_usage/MB Memory/RAM_estimated/MB Memory/Distance_travelled/m "$SKIP/$d/loc"
  $REPORT_TOOL --export --export_prefix "Consecutive$d" --loc 32 Loop/Map_id/ "$SKIP/$d/consecutive_loc"
done


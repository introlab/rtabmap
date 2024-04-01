#!/bin/bash

if [ $# -lt 1 ]
then
  echo "No arguments supplied. It should be the data directory (where the reprocessed map databases are saved)."
  exit
fi
DATA=$1

DETECTOR=(0 1 6 7 9 14 11 111) 

source rtabmap_latest.bash

for d in "${DETECTOR[@]}"
do
  rtabmap-report --export --export_prefix "Stat$d" --loc 32  Loop/Odom_correction_norm/m Loop/Visual_inliers/ Timing/Total/ms Timing/Proximity_by_space_visual/ms Timing/Likelihood_computation/ms Timing/Posterior_computation/ms TimingMem/Keypoints_detection/ms TimingMem/Descriptors_extraction/ms TimingMem/Add_new_words/ms Loop/Map_id/ Keypoint/Current_frame/words Memory/RAM_usage/MB Memory/RAM_estimated/MB Memory/Distance_travelled/m Loop/Distance_since_last_loc/ Memory/Local_graph_size/ Keypoint/Dictionary_size/words "$DATA/$d/loc"
  rtabmap-report --export --export_prefix "Consecutive$d" --loc 32 Loop/Map_id/ Loop/Distance_since_last_loc/ "$DATA/$d/consecutive_loc"
done


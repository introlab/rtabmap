#!/bin/bash

if [ $# -eq 0 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint)."
  exit
fi
TYPE=$1

SKIP=0
if [ $# -eq 2 ]
then
  SKIP=$2
fi

PREFIX="/home/mathieu/workspace/rtabmap_cv_latest/bin/"
REPROCESS_TOOL="${PREFIX}rtabmap-reprocess"

# 'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_9999.db' 'merged_135.db' 'merged_246.db' 'merged_16.db' 'merged_9999_reduced.db'
DATABASES=(  'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_9999.db' 'merged_135.db' 'merged_246.db' 'merged_16.db'  )
# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db"

[ ! -d "$SKIP/$TYPE/loc" ] && mkdir $SKIP/$TYPE/loc

echo $PARAMS
for db in "${DATABASES[@]}"
do
  $REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/loc/loc_$db
done



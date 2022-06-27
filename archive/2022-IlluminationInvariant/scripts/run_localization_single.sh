#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint) and the data directory (where map databases have been reprocessed)."
  exit
fi
TYPE=$1
DATA=$2

source rtabmap_latest.bash

# 'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_123456.db' 'merged_135.db' 'merged_246.db' 'merged_16.db' 'merged_123456_reduced.db'
DATABASES=('map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_123456.db' 'merged_135.db' 'merged_246.db' 'merged_16.db' 'merged_123456_reduced.db')
# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db"

[ ! -d "$DATA/$TYPE/loc" ] && mkdir $DATA/$TYPE/loc

echo $PARAMS
for db in "${DATABASES[@]}"
do
  rtabmap-reprocess -loc_null --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --RGBD/ProximityMaxPaths 1 --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --uwarn "$DATA/$TYPE/$db;$LOCALIZATION_DATABASES" $DATA/$TYPE/loc/loc_$db
done



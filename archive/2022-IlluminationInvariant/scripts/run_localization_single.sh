#!/bin/bash

if [ $# -lt 3 ]
then
  echo "No arguments supplied. They should be 3: the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint), the input directory (original maps) and the output data directory (where reprocessed map databases will be saved)."
  exit
fi
TYPE=$1
INPUT=$2
OUTPUT=$3


source rtabmap_latest.bash

# 'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_123456.db' 'merged_135.db' 'merged_246.db' 'merged_16.db' 'merged_123456_reduced.db'
DATABASES=('map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' 'merged_123456.db' 'merged_135.db' 'merged_246.db' 'merged_16.db' 'merged_123456_reduced.db')
# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="$INPUT/loc_190321-165128.db;$INPUT/loc_190321-173134.db;$INPUT/loc_190321-175823.db;$INPUT/loc_190321-183051.db;$INPUT/loc_190321-185950.db;$INPUT/loc_190321-194226.db"

[ ! -d "$OUTPUT/$TYPE/loc" ] && mkdir $OUTPUT/$TYPE/loc

echo $PARAMS
for db in "${DATABASES[@]}"
do
  rtabmap-reprocess -loc_null --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --RGBD/ProximityMaxPaths 1 --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/loc/loc_$db
done



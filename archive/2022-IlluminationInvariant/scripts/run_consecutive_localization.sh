#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint) and the data directory (where map databases have been reprocessed)."
  exit
fi
TYPE=$1
DATA=$2

source rtabmap_latest.bash

SOURCE=('map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db')
TARGETS=($DATA/$TYPE'/map_190321-172717.db;'$DATA/$TYPE'/map_190321-175428.db;'$DATA/$TYPE'/map_190321-193556.db' $DATA/$TYPE'/map_190321-175428.db;'$DATA/$TYPE'/map_190321-182709.db;' $DATA/$TYPE'/map_190321-182709.db;'$DATA/$TYPE'/map_190321-185608.db' $DATA/$TYPE'/map_190321-185608.db;'$DATA/$TYPE'/map_190321-193556.db' $DATA/$TYPE'/map_190321-193556.db' )


[ ! -d "$DATA/$TYPE/consecutive_loc" ] && mkdir $DATA/$TYPE/consecutive_loc

for i in ${!SOURCE[@]}
do
    db=${SOURCE[$i]}
    loc_dbs=${TARGETS[$i]}
    rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --RGBD/ProximityMaxPaths 1 --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --uwarn "$DATA/$TYPE/$db;$loc_dbs" $DATA/$TYPE/consecutive_loc/loc_$db
done


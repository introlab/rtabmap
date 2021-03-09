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

SOURCE=('map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db')
TARGETS=($SKIP/$TYPE'/map_190321-172717.db;'$SKIP/$TYPE'/map_190321-175428.db;'$SKIP/$TYPE'/map_190321-193556.db' $SKIP/$TYPE'/map_190321-175428.db;'$SKIP/$TYPE'/map_190321-182709.db;' $SKIP/$TYPE'/map_190321-182709.db;'$SKIP/$TYPE'/map_190321-185608.db' $SKIP/$TYPE'/map_190321-185608.db;'$SKIP/$TYPE'/map_190321-193556.db' $SKIP/$TYPE'/map_190321-193556.db' )


[ ! -d "$SKIP/$TYPE/consecutive_loc" ] && mkdir $SKIP/$TYPE/consecutive_loc

for i in ${!SOURCE[@]}
do
    db=${SOURCE[$i]}
    loc_dbs=${TARGETS[$i]}
    $REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --uwarn "$SKIP/$TYPE/$db;$loc_dbs" $SKIP/$TYPE/consecutive_loc/loc_$db
done


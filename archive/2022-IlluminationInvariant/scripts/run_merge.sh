#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint) and the data directory (where map databases have been reprocessed)."
  exit
fi
TYPE=$1
DATA=$2

MIN_INLIERS=20 #20 40 60 80

source rtabmap_latest.bash

DATABASES="$DATA/$TYPE/map_190321-164651.db;$DATA/$TYPE/map_190321-172717.db;$DATA/$TYPE/map_190321-175428.db;$DATA/$TYPE/map_190321-182709.db;$DATA/$TYPE/map_190321-185608.db;$DATA/$TYPE/map_190321-193556.db"

# To compute "Ground truth"
rtabmap-reprocess --uwarn "$DATABASES" $DATA/$TYPE/merged_123456.db

cp $DATA/$TYPE/merged_123456.db $DATA/$TYPE/merged_123456_gt.db
rtabmap-detectMoreLoopClosures -r 0.5 -i 5 $DATA/$TYPE/merged_123456_gt.db

rtabmap-reprocess --uwarn -gt $DATA/$TYPE/merged_123456_gt.db $DATA/$TYPE/merged_123456.db

rtabmap-reprocess --uwarn "$DATA/$TYPE/map_190321-164651.db;$DATA/$TYPE/map_190321-193556.db" $DATA/$TYPE/merged_16.db

rtabmap-reprocess --uwarn "$DATA/$TYPE/map_190321-164651.db;$DATA/$TYPE/map_190321-175428.db;$DATA/$TYPE/map_190321-185608.db" $DATA/$TYPE/merged_135.db

rtabmap-reprocess --uwarn "$DATA/$TYPE/map_190321-172717.db;$DATA/$TYPE/map_190321-182709.db;$DATA/$TYPE/map_190321-193556.db" $DATA/$TYPE/merged_246.db

# Reduced graph
rtabmap-reprocess --uwarn -gt --Mem/ReduceGraph true --Vis/MinInliers $MIN_INLIERS $DATA/$TYPE/merged_123456_gt.db $DATA/$TYPE/merged_123456_reduced.db



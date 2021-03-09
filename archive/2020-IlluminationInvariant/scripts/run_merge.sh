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
DETECT_MORE_LOOP_CLOSURE_TOOL="${PREFIX}rtabmap-detectMoreLoopClosures"

DATABASES="$SKIP/$TYPE/map_190321-164651.db;$SKIP/$TYPE/map_190321-172717.db;$SKIP/$TYPE/map_190321-175428.db;$SKIP/$TYPE/map_190321-182709.db;$SKIP/$TYPE/map_190321-185608.db;$SKIP/$TYPE/map_190321-193556.db"

$REPROCESS_TOOL --uwarn --RGBD/OptimizeMaxError 0 "$DATABASES" $SKIP/$TYPE/merged_9999.db
$DETECT_MORE_LOOP_CLOSURE_TOOL $SKIP/$TYPE/merged_9999.db

#$REPROCESS_TOOL --uwarn --RGBD/OptimizeMaxError 0 --Mem/ReduceGraph true --Vis/MinInliers 60 "$DATABASES" $SKIP/$TYPE/merged_9999_reduced.db
#$DETECT_MORE_LOOP_CLOSURE_TOOL $SKIP/$TYPE/merged_9999_reduced.db

$REPROCESS_TOOL --uwarn --RGBD/OptimizeMaxError 0 "$SKIP/$TYPE/map_190321-164651.db;$SKIP/$TYPE/map_190321-193556.db" $SKIP/$TYPE/merged_16.db
$DETECT_MORE_LOOP_CLOSURE_TOOL $SKIP/$TYPE/merged_16.db

$REPROCESS_TOOL --uwarn --RGBD/OptimizeMaxError 0 "$SKIP/$TYPE/map_190321-164651.db;$SKIP/$TYPE/map_190321-175428.db;$SKIP/$TYPE/map_190321-185608.db" $SKIP/$TYPE/merged_135.db
$DETECT_MORE_LOOP_CLOSURE_TOOL $SKIP/$TYPE/merged_135.db

$REPROCESS_TOOL --uwarn --RGBD/OptimizeMaxError 0 "$SKIP/$TYPE/map_190321-172717.db;$SKIP/$TYPE/map_190321-182709.db;$SKIP/$TYPE/map_190321-193556.db" $SKIP/$TYPE/merged_246.db
$DETECT_MORE_LOOP_CLOSURE_TOOL $SKIP/$TYPE/merged_246.db




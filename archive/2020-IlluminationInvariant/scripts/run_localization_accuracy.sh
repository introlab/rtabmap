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

# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db"

[ ! -d "$SKIP/$TYPE/loc" ] && mkdir $SKIP/$TYPE/accuracy

db=merged_9999.db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOn_$db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 1 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOn_$db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOff_$db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 0 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOff_$db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOn_$db

$REPROCESS_TOOL --skip $SKIP --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "$SKIP/$TYPE/$db;$LOCALIZATION_DATABASES" $SKIP/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOff_$db



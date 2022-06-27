#!/bin/bash

if [ $# -eq 0 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint)."
  exit
fi
TYPE=$1

PREFIX="/home/mathieu/workspace/rtabmap_cv_latest/bin/"
REPROCESS_TOOL="${PREFIX}rtabmap-reprocess"

# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db"

[ ! -d "results/$TYPE/loc" ] && mkdir results/$TYPE/accuracy

db=merged_9999.db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOn_$db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 1 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOn_$db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOff_$db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 0 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOff_$db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOn_$db

$REPROCESS_TOOL --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "results/$TYPE/$db;$LOCALIZATION_DATABASES" results/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOff_$db



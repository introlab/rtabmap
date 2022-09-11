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

# loc_190321-165128.db;loc_190321-173134.db;loc_190321-175823.db;loc_190321-183051.db;loc_190321-185950.db;loc_190321-194226.db
LOCALIZATION_DATABASES="$INPUT/loc_190321-165128.db;$INPUT/loc_190321-173134.db;$INPUT/loc_190321-175823.db;$INPUT/loc_190321-183051.db;$INPUT/loc_190321-185950.db;$INPUT/loc_190321-194226.db"

[ ! -d "$OUTPUT/$TYPE/loc" ] && mkdir $OUTPUT/$TYPE/accuracy

db=merged_123456.db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOn_$db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 1 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOn_$db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOff_DoubleRegOn_BaOff_$db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess false --Reg/RepeatOnce false --Vis/BundleAdjustment 0 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOff_DoubleRegOff_BaOff_$db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 1 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOn_$db

rtabmap-reprocess --Mem/IncrementalMemory false --RGBD/ProximityBySpace true --Mem/LocalizationDataSaved true --Mem/BinDataKept false --RGBD/SavedLocalizationIgnored true --Kp/IncrementalFlann false --Vis/MinInliers 20 --Rtabmap/PublishRAMUsage true --RGBD/ProximityOdomGuess true --Reg/RepeatOnce true --Vis/BundleAdjustment 0 --uwarn "$OUTPUT/$TYPE/$db;$LOCALIZATION_DATABASES" $OUTPUT/$TYPE/accuracy/ProxOn_DoubleRegOn_BaOff_$db



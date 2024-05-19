#!/bin/bash

DATASET_FOLDER=""
GT_FILE=""
if [ $# -eq 2 ] 
then
    DATASET_FOLDER=$1
    GT_FILE=$2
else
echo "Usage: run_bow.sh \"dataset folder\" \"ground truth file\""
exit
fi

rtabmap-console \
       -quiet \
       --Rtabmap/StatisticLogged true\
       --Rtabmap/StatisticLoggedHeaders false\
       --Kp/DetectorStrategy 0\
       --SURF/HessianThreshold 150\
       --Rtabmap/MemoryThr 300\
       --Rtabmap/LoopRatio 0.9\
       --Mem/STMSize 30\
       --Vis/MaxFeatures 400\
       --Kp/TfIdfLikelihoodUsed false\
       --Kp/MaxFeatures 400\
       --Kp/BadSignRatio 0.25\
       --Mem/BadSignaturesIgnored true\
       --Mem/RehearsalSimilarity 0.20\
       --Mem/RecentWmRatio 0.20\
       -gt "$GT_FILE"\
       "$DATASET_FOLDER"


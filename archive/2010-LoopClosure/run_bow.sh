#!/bin/bash

DATASET_FOLDER=""
GT_FILE=""
MEMORY_THR=300
DETECTOR=0
GPU=false
if [ $# -ge 2 ] 
then
    DATASET_FOLDER=$1
    GT_FILE=$2
else
echo "Usage: run_bow.sh \"dataset folder\" \"ground truth file\" [\"Rtabmap/MemoryThr=300\"] [\"Kp/DetectorStrategy=0\"] [\"GPU=false\"]"
exit
fi

if [ $# -ge 3 ] 
then
    MEMORY_THR=$3
fi
if [ $# -ge 4 ] 
then
    DETECTOR=$4
fi 
if [ $# -ge 5 ] 
then
    GPU=$5
fi 

rtabmap-console \
       -quiet \
       --Rtabmap/StatisticLogged true\
       --Rtabmap/StatisticLoggedHeaders false\
       --Kp/DetectorStrategy $DETECTOR\
       --SURF/HessianThreshold 150\
       --Rtabmap/MemoryThr $MEMORY_THR\
       --Rtabmap/LoopRatio 0.9\
       --Mem/STMSize 30\
       --Vis/MaxFeatures 400\
       --Kp/TfIdfLikelihoodUsed false\
       --Kp/MaxFeatures 400\
       --Kp/BadSignRatio 0.25\
       --Mem/BadSignaturesIgnored true\
       --Mem/RehearsalSimilarity 0.20\
       --Mem/RecentWmRatio 0.20\
       --FAST/Gpu $GPU\
       --GFTT/Gpu $GPU\
       --ORB/Gpu $GPU\
       --SIFT/Gpu $GPU\
       --SURF/GpuVersion $GPU\
       -gt "$GT_FILE"\
       "$DATASET_FOLDER"


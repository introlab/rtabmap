#!/bin/bash

DATASET_FOLDER=""
GT_FILE=""
MEMORY_THR=0
if [ $# -ge 2 ] 
then
    DATASET_FOLDER=$1
    GT_FILE=$2
else
echo "Usage: run_all_detectors.sh \"dataset folder\" \"ground truth file\" [\"Rtabmap/MemoryThr=0\"]"
exit
fi

if [ $# -ge 3 ] 
then
    MEMORY_THR=$3
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

$SCRIPT_DIR/run_bow.sh $DATASET_FOLDER $GT_FILE $MEMORY_THR 0 false
mv LogF.txt $DATASET_FOLDER/SurfLogF.txt
mv LogI.txt $DATASET_FOLDER/SurfLogI.txt
$SCRIPT_DIR/run_bow.sh $DATASET_FOLDER $GT_FILE $MEMORY_THR 1 false
mv LogF.txt $DATASET_FOLDER/SiftLogF.txt
mv LogI.txt $DATASET_FOLDER/SiftLogI.txt
$SCRIPT_DIR/run_bow.sh $DATASET_FOLDER $GT_FILE $MEMORY_THR 1 true
mv LogF.txt $DATASET_FOLDER/CudaSiftLogF.txt
mv LogI.txt $DATASET_FOLDER/CudaSiftLogI.txt
$SCRIPT_DIR/run_bow.sh $DATASET_FOLDER $GT_FILE $MEMORY_THR 6 false
mv LogF.txt $DATASET_FOLDER/GfttBriefLogF.txt
mv LogI.txt $DATASET_FOLDER/GfttBriefLogI.txt
#$SCRIPT_DIR/run_bow.sh $DATASET_FOLDER $GT_FILE $MEMORY_THR 11 false
#mv LogF.txt $DATASET_FOLDER/SuperPointLogF.txt
#mv LogI.txt $DATASET_FOLDER/SuperPointLogI.txt
 

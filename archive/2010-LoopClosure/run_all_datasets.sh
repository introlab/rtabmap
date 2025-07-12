#!/bin/bash

DATASETS_FOLDER=""
if [ $# -eq 1 ] 
then
    DATASETS_FOLDER=$1
else
echo "Usage: run_all_datasets.sh \"datasets folder\" "
exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

$SCRIPT_DIR/run_bow.sh $DATASETS_FOLDER/NewCollege $DATASETS_FOLDER/NewCollege.png
if [ -s LogF.txt ]; then
mv LogF.txt $DATASETS_FOLDER/NewCollege/LogF.txt
mv LogI.txt $DATASETS_FOLDER/NewCollege/LogI.txt
fi
$SCRIPT_DIR/run_bow.sh $DATASETS_FOLDER/CityCentre $DATASETS_FOLDER/CityCentre.png
if [ -s LogF.txt ]; then
mv LogF.txt $DATASETS_FOLDER/CityCentre/LogF.txt
mv LogI.txt $DATASETS_FOLDER/CityCentre/LogI.txt
fi
$SCRIPT_DIR/run_bow.sh $DATASETS_FOLDER/UdeS_1Hz $DATASETS_FOLDER/UdeS_1Hz.png
if [ -s LogF.txt ]; then
mv LogF.txt $DATASETS_FOLDER/UdeS_1Hz/LogF.txt
mv LogI.txt $DATASETS_FOLDER/UdeS_1Hz/LogI.txt
fi
 

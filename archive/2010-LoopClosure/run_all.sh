#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

$SCRIPT_DIR/run_bow.sh ~/loop_closure_detection_datasets/NewCollege ~/loop_closure_detection_datasets/NewCollege.png
$SCRIPT_DIR/run_bow.sh ~/loop_closure_detection_datasets/CityCentre ~/loop_closure_detection_datasets/CityCentre.png
$SCRIPT_DIR/run_bow.sh ~/loop_closure_detection_datasets/UdeS_1Hz ~/loop_closure_detection_datasets/UdeS_1Hz.png
 

#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. They should be 2: the input directory (original maps) and the output data directory (where reprocessed map databases will be saved)."
  exit
fi
INPUT=$1
OUTPUT=$2

./reprocess_maps_all.sh $INPUT $OUTPUT
./run_localization_single_all.sh $INPUT $OUTPUT
./run_consecutive_localization_all.sh $OUTPUT


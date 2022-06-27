#!/bin/bash

if [ $# -lt 1 ]
then
  echo "No arguments supplied. It should be the data directory (where the reprocessed map databases will be saved)."
  exit
fi
DATA=$1

./reprocess_maps_all.sh $DATA
./run_localization_single_all.sh $DATA
./run_consecutive_localization_all.sh $DATA


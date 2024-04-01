#!/bin/bash

if [ $# -lt 1 ]
then
  echo "No arguments supplied. It should be the data directory (where the reprocessed map databases will be saved)."
  exit
fi
DATA=$1

DETECTOR=(0 1 6 7 9 14 11 111)

for d in "${DETECTOR[@]}"
do
  ./run_consecutive_localization.sh $d $DATA
done


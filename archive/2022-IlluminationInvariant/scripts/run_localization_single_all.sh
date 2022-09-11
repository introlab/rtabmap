#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. They should be 2: the input directory (original maps) and the output data directory (where reprocessed map databases will be saved)."
  exit
fi
INPUT=$1
OUTPUT=$2

DETECTOR=(0 1 6 7 9 14 11 111)

for d in "${DETECTOR[@]}"
do
  ./run_localization_single.sh $d $INPUT $OUTPUT
done


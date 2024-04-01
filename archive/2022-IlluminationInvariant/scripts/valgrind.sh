#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. They should be 2: the input directory (original maps) and the output data directory (where reprocessed map databases will be saved)."
  exit
fi
INPUT=$1
OUTPUT=$2

DETECTOR=(0 1 6 7 9 14 11 111)

source rtabmap_latest.bash

for d in "${DETECTOR[@]}"
do
  valgrind --tool=massif --time-unit=ms --detailed-freq=1 --max-snapshots=100 rtabmap-reprocess --Mem/IncrementalMemory false --Kp/IncrementalFlann false  "${OUTPUT}/${d}/map_190321-164651.db;${INPUT}/loc_190321-165128.db" output.db
  rm output.db
done


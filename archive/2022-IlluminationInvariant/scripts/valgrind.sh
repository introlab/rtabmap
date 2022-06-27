#!/bin/bash

if [ $# -lt 1 ]
then
  echo "No arguments supplied. It should be the data directory (where the reprocessed map databases are saved)."
  exit
fi
DATA=$1

DETECTOR=(0 1 6 7 9 14 11 111)

source rtabmap_latest.bash

for d in "${DETECTOR[@]}"
do
  valgrind --tool=massif --time-unit=ms --detailed-freq=1 --max-snapshots=100 rtabmap-reprocess --Mem/IncrementalMemory false --Kp/IncrementalFlann false  "${DATA}/${d}/map_190321-164651.db;loc_190321-165128.db" output.db
  rm output.db
done


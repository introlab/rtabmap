#!/bin/bash

SKIP=0
if [ $# -eq 1 ]
then
  SKIP=$1
fi

DETECTOR=(0 1 6 7 8 9 11 12 14)

PREFIX="/home/mathieu/workspace/rtabmap_cv_latest/bin/"
REPORT_TOOL="${PREFIX}rtabmap-report"

for d in "${DETECTOR[@]}"
do
  valgrind --tool=massif --time-unit=ms --detailed-freq=1 --max-snapshots=100 ${PREFIX}rtabmap-reprocess --Mem/IncrementalMemory false --Kp/IncrementalFlann false  "${SKIP}/${d}/merged_9999.db;map_190321-164651.db" output.db
  rm output.db
done


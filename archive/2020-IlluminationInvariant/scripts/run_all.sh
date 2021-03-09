#!/bin/bash

SKIP=0
if [ $# -eq 1 ]
then
  SKIP=$1
fi

./reprocess_maps_all.sh $SKIP
./run_merge.sh $SKIP
./run_localization_single_all.sh $SKIP
./run_consecutive_localization_all.sh $SKIP


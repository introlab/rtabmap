#!/bin/bash

SKIP=0
if [ $# -eq 1 ]
then
  SKIP=$1
fi

DETECTOR=(0 1 6 7 9 11 12 14)

for d in "${DETECTOR[@]}"
do
  ./run_consecutive_localization.sh $d $SKIP
done


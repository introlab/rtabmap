#!/bin/bash

if [ $# -eq 0 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint)."
  exit
fi
TYPE=$1

SKIP=0
if [ $# -eq 2 ]
then
  SKIP=$2
fi

PREFIX="/home/mathieu/workspace/rtabmap_cv_latest/bin/"
REPROCESS_TOOL="${PREFIX}rtabmap-reprocess"
DETECT_MORE_LOOP_CLOSURE_TOOL="${PREFIX}rtabmap-detectMoreLoopClosures"

[ ! -d "$SKIP" ] && mkdir $SKIP
[ ! -d "$SKIP/$TYPE" ] && mkdir $SKIP/$TYPE
# 'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db'
DATABASES=( 'map_190321-164651.db' 'map_190321-172717.db' 'map_190321-175428.db' 'map_190321-182709.db' 'map_190321-185608.db' 'map_190321-193556.db' )

PARAMS="--Kp/DetectorStrategy $TYPE --Vis/FeatureType $TYPE"

if [ $TYPE -eq 2 ] || [ $TYPE -eq 3 ] || [ $TYPE -eq 4 ] || [ $TYPE -eq 5 ] || [ $TYPE -eq 6 ] || [ $TYPE -eq 7 ] || [ $TYPE -eq 8 ] || [ $TYPE -eq 10 ] || [ $TYPE -eq 12 ]
then
  # binary descriptors
  PARAMS="--Vis/CorNNDR 0.8 $PARAMS"
else
  # float descriptors
  PARAMS="--Vis/CorNNDR 0.6 $PARAMS"
if

echo $PARAMS
for db in "${DATABASES[@]}"
do
  $REPROCESS_TOOL --skip $SKIP --RGBD/MarkerDetection false --RGBD/ProximityBySpace true --RGBD/LocalRadius 1 --Mem/InitWMWithAllNodes true --Rtabmap/TimeThr 0 --Mem/UseOdomFeatures false --Optimizer/GravitySigma 0.1 --Mem/UseOdomGravity true --RGBD/OptimizeFromGraphEnd false --Mem/DepthAsMask false --RGBD/OptimizeMaxError 4 --RGBD/ProximityOdomGuess false --Vis/MaxFeatures 1000 --Kp/MaxFeatures 400 --Vis/EpipolarGeometryVar 0.1 --Vis/EstimationType 1 --Vis/MinInliers 20 --Rtabmap/MaxRetrieved 2 --Optimizer/Iterations 20 --Mem/CompressionParallelized true --Kp/Parallelized true --Kp/MaxDepth 0 --Kp/BadSignRatio 0.2 --BRIEF/Bytes 32 --Kp/ByteToFloat true --SURF/HessianThreshold 100 --SIFT/ContrastThreshold 0.02 --BRISK/Thresh 10 --SuperPoint/ModelPath superpoint.pt --Rtabmap/PublishRAMUsage true --ORB/EdgeThreshold 19 --ORB/ScaleFactor 2 --ORB/NLevels 3 --uerror $PARAMS $db $SKIP/$TYPE/$db
  $DETECT_MORE_LOOP_CLOSURE_TOOL --uwarn $SKIP/$TYPE/$db
done


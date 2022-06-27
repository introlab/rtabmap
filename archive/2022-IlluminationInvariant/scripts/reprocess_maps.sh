#!/bin/bash

if [ $# -lt 2 ]
then
  echo "No arguments supplied. It should be the detector number type (0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint) and the data directory (where reprocessed map databases will be saved)."
  exit
fi
TYPE=$1
DATA=$2

source rtabmap_latest.bash

[ ! -d "$DATA" ] && mkdir $DATA
[ ! -d "$DATA/$TYPE" ] && mkdir $DATA/$TYPE
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
fi

if [ $TYPE -eq 111 ] 
then
  PARAMS="--Vis/CorNNType 6 --SuperGlue/Path ~/workspace/SuperGluePretrainedNetwork/rtabmap_superglue.py --Reg/RepeatOnce false --Vis/CorGuessWinSize 0 $PARAMS --Kp/DetectorStrategy 11 --Vis/FeatureType 11"
fi

echo $PARAMS
for db in "${DATABASES[@]}"
do
  rtabmap-reprocess --RGBD/MarkerDetection false --RGBD/ProximityBySpace true --RGBD/LocalRadius 1 --Mem/InitWMWithAllNodes true --Rtabmap/TimeThr 0 --Mem/UseOdomFeatures false --Optimizer/GravitySigma 0.1 --Mem/UseOdomGravity true --RGBD/OptimizeFromGraphEnd false --Mem/DepthAsMask false --RGBD/OptimizeMaxError 10 --RGBD/ProximityOdomGuess false --Vis/MaxFeatures 1000 --Kp/MaxFeatures 400 --Vis/EpipolarGeometryVar 0.1 --Vis/EstimationType 1 --Vis/MinInliers 20 --Rtabmap/MaxRetrieved 2 --Optimizer/Iterations 20 --Mem/CompressionParallelized true --Kp/Parallelized true --Kp/MaxDepth 0 --Kp/BadSignRatio 0.2 --BRIEF/Bytes 32 --Kp/ByteToFloat true --SURF/HessianThreshold 100 --SIFT/ContrastThreshold 0.02 --BRISK/Thresh 10 --SuperPoint/ModelPath superpoint.pt --Rtabmap/PublishRAMUsage true --ORB/EdgeThreshold 19 --ORB/ScaleFactor 2 --ORB/NLevels 3 --Db/TargetVersion "" --Icp/CorrespondenceRatio 0.1 --RGBD/MaxOdomCacheSize 0 --uerror $PARAMS $db $DATA/$TYPE/$db
  rtabmap-detectMoreLoopClosures --uwarn $DATA/$TYPE/$db
done


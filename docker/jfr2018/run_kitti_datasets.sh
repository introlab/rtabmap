#!/bin/bash

PWD=$(pwd)
DOCKER=1 # set to 0 to use host rtabmap tool and relative datasets to this script

KITTI_ROOT_PATH='./datasets/kitti'
KITTI_RESULTS_PATH='./results/kitti'
mkdir -p $KITTI_RESULTS_PATH
if [ $DOCKER -eq 1 ]
then
    KITTI_ROOT_PATH='/root/datasets/kitti'
    KITTI_RESULTS_PATH='/root/results/kitti'
fi

LIST=('00' '01' '02' '03' '04' '05' '06' '07' '08' '09' '10')
#LIST=('11' '12' '13' '14' '15' '16' '17' '18' '19' '20' '21')

STRATEGY=0
REG=0
OUTPUT="test"
if [ $# -eq 4 ] 
then
    OUTPUT=$1
    STRATEGY=$2
    REG=$3
    LIST=($4)
elif [ $# -eq 3 ]
then
    OUTPUT=$1
    STRATEGY=$2
    REG=$3
else
echo "Usage: run_kitti_datasets.sh \"output name\" \"odom strategy: 0=f2m 1=f2f 11=f2f_optflow 2=fovis 3=viso2 4=dvo 5=orbslam2 7=LOAM 9=VINS\" \"reg strategy: 0=vis 1=icp-point2point 11=icp-point2plane\" [sequence]"
exit
fi

F2F_params=""
if [ $STRATEGY -eq 11 ]
then
    STRATEGY=1
    F2F_params="--Vis/CorType 1"
fi

if [ $STRATEGY -eq 1 ]
then
    F2F_params="--Odom/KeyFrameThr 0.6 $F2F_params"
fi

if [ $STRATEGY -eq 3 ]
then
    F2F_params=" $F2F_params"
fi

if [ $STRATEGY -eq 9 ]
then
    F2F_params="--OdomVINS/ConfigPath ./kitti_config04-12.yaml $F2F_params"
fi

SCAN=""
if [[ $REG -eq 1 || $REG -eq 11 ]]
then
   if [ $REG -eq 11 ]
   then
      REG=1
      if [ $STRATEGY -eq 7 ]
      then
         SCAN="--Mem/LaserScanVoxelSize 0.5 --Mem/LaserScanNormalK 10 --scan_voxel 0.0"
      else
         SCAN="--scan_k 10 --scan_voxel 0.5"
      fi
      SCAN="--Icp/PointToPlane true --Icp/PointToPlaneK 10 --Icp/PointToPlaneRadius 0 --Icp/Iterations 5 --Odom/ScanKeyFrameThr 0.6 $SCAN"
   else
      if [ $STRATEGY -eq 7 ]
      then
         SCAN="--Mem/LaserScanVoxelSize 0.5 --scan_voxel 0.0"
      else
         SCAN="--scan_voxel 0.5"
      fi
      SCAN="--Icp/PointToPlane false --Icp/Iterations 10 --Odom/ScanKeyFrameThr 0.8 $SCAN"
   fi
   SCAN="--scan --Reg/Strategy $REG --OdomF2M/ScanMaxSize 10000 --OdomF2M/ScanSubtractRadius 0.5 --Odom/LOAMSensor 2 --Icp/MaxTranslation 2 --Icp/Epsilon 0.0001 --Icp/MaxCorrespondenceDistance 1.5 --Icp/CorrespondenceRatio 0.01 --Icp/PM true --Icp/PMOutlierRatio 0.7 --Icp/PMMatcherKnn 3 --Icp/PMMatcherEpsilon 1 --Icp/ReciprocalCorrespondences false  $SCAN"
fi

RTABMAP_KITTI_TOOL="rtabmap-kitti_dataset"
if [ $DOCKER -eq 1 ]
then
   # Select rtabmap built with os2 support or not
   TOOL_PREFIX="/usr/local/bin"
   if [ $STRATEGY -eq 5 ]
   then
       TOOL_PREFIX="/root/rtabmap_os2/bin"
   elif [ $STRATEGY -eq 7 ]
   then
       TOOL_PREFIX="/root/rtabmap_loam/bin"
   fi
   RTABMAP_KITTI_TOOL="docker run -v $PWD/datasets/kitti:$KITTI_ROOT_PATH -v $PWD/results/kitti:$KITTI_RESULTS_PATH -i -t --rm introlab3it/rtabmap:jfr2018 $TOOL_PREFIX/rtabmap-kitti_dataset"
fi
echo $RTABMAP_KITTI_TOOL

for d in "${LIST[@]}"
do
    taskset 0x1 $RTABMAP_KITTI_TOOL \
       --Rtabmap/PublishRAMUsage true\
       --Vis/MaxFeatures 1500\
       --RGBD/LinearUpdate 0\
       --RGBD/AngularUpdate 0\
       --RGBD/ProximityBySpace false\
       --Rtabmap/DetectionRate 2\
       --Kp/MaxFeatures 750\
       --Kp/FlannRebalancingFactor 1.0\
       --GFTT/QualityLevel 0.01\
       --GFTT/MinDistance 7\
       --OdomF2M/MaxSize 3000\
       --Rtabmap/CreateIntermediateNodes true\
       --Odom/Strategy $STRATEGY\
       $F2F_params \
       --OdomORBSLAM2/VocPath /root/ORBvoc.txt\
       --OdomORBSLAM2/ThDepth 40\
       --OdomORBSLAM2/Fps 10\
       --OdomORBSLAM2/MaxFeatures 2000\
       --OdomViso2/MatchNmsN 7\
       --OdomViso2/MatchRadius 300\
       --FAST/Threshold 12\
       --Mem/STMSize 30\
       --Mem/UseOdomFeatures false \
       --Mem/BinDataKept false \
       --Vis/CorNNDR 0.6 \
       --Vis/CorGuessWinSize 20 \
       $SCAN \
       --gt $KITTI_ROOT_PATH"/devkit/cpp/data/odometry/poses/$d.txt"\
       --output "$KITTI_RESULTS_PATH/$d"\
       --output_name $OUTPUT\
       --quiet\
       $KITTI_ROOT_PATH/dataset/sequences/$d
done

# ORB_SLAM2:
# 0-2: --OdomORBSLAM2/ThDepth 35
# 3-12: --OdomORBSLAM2/ThDepth 40 (default)
# 0-3: --FAST/Threshold 20 (default)
# 4-12: --FAST/Threshold 12 --OdomORBSLAM2/ThDepth 40
# --OdomORBSLAM2/MaxFeatures 2000
# 


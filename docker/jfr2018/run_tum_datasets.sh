#!/bin/bash

PWD=$(pwd)
DOCKER=1 # set to 0 to use host rtabmap tool and relative datasets to this script

RGBD_ROOT_PATH='./datasets/tum'
RGBD_RESULTS_PATH='./results/tum'
mkdir -p $RGBD_RESULTS_PATH
if [ $DOCKER -eq 1 ]
then
    RGBD_ROOT_PATH='/root/datasets/tum'
RGBD_RESULTS_PATH='/root/results/tum'
fi

LIST=( 'rgbd_dataset_freiburg1_desk' 'rgbd_dataset_freiburg1_desk2' 'rgbd_dataset_freiburg1_room' 'rgbd_dataset_freiburg2_desk' 'rgbd_dataset_freiburg2_xyz' 'rgbd_dataset_freiburg3_long_office_household' 'rgbd_dataset_freiburg3_nostructure_texture_near_withloop_validation')
# more: 'rgbd_dataset_freiburg1_360' 'rgbd_dataset_freiburg1_plant' 'rgbd_dataset_freiburg1_rpy' 'rgbd_dataset_freiburg1_teddy' 'rgbd_dataset_freiburg2_360_hemisphere' 'rgbd_dataset_freiburg2_large_no_loop' 'rgbd_dataset_freiburg2_large_with_loop' 'rgbd_dataset_freiburg1_floor'
# Got Lost: 'rgbd_dataset_freiburg1_floor' (at 11 seconds end), 'rgbd_dataset_freiburg3_teddy'

STRATEGY=0
OUTPUT="test"
if [ $# -eq 3 ] 
then
    OUTPUT=$1
    STRATEGY=$2
    LIST=($3)
elif [ $# -eq 2 ] 
then
    OUTPUT=$1
    STRATEGY=$2
else
echo "Usage: run_tum_datasets.sh \"output name\" \"odom strategy: 0=f2m 1=f2f 11=f2f_optflow 2=fovis 3=viso2 4=dvo 5=orbslam2\" [sequence]"
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

if [[ $STRATEGY -eq 2 || $STRATEGY -eq 4 ]]
then
    F2F_params="--RGBD/OptimizeMaxError 0 $F2F_params"
fi

RTABMAP_RGBD_TOOL="rtabmap-rgbd_dataset"
if [ $DOCKER -eq 1 ]
then
    # Select rtabmap built with os2 support or not
    TOOL_PREFIX="/usr/local/bin"
    if [ $STRATEGY -eq 5 ]
    then
        TOOL_PREFIX="/root/rtabmap_os2/bin"
    fi
    RTABMAP_RGBD_TOOL="docker run -v $PWD/datasets/tum:$RGBD_ROOT_PATH -v $PWD/results/tum:$RGBD_RESULTS_PATH -i -t --rm introlab3it/rtabmap:jfr2018 $TOOL_PREFIX/rtabmap-rgbd_dataset"
fi
echo $RTABMAP_RGBD_TOOL

# no --disp with 01?
for d in "${LIST[@]}"
do

    DETECTION_RATE=2
    if [[ "$d" == "rgbd_dataset_freiburg1_desk" || "$d" == "rgbd_dataset_freiburg1_desk2" ]]
    then
        DETECTION_RATE=4
    fi

    taskset 0x1 $RTABMAP_RGBD_TOOL \
       --Rtabmap/PublishRAMUsage true\
       --RGBD/LinearUpdate 0\
       --RGBD/AngularUpdate 0\
       --Rtabmap/DetectionRate $DETECTION_RATE\
       --Kp/FlannRebalancingFactor 1.0\
       --Odom/Strategy $STRATEGY\
       --Mem/STMSize 30\
       --Mem/UseOdomFeatures false \
       --Mem/BinDataKept true \
       --Rtabmap/CreateIntermediateNodes false\
       $F2F_params\
       --OdomORBSLAM2/VocPath /root/ORBvoc.txt\
       --OdomFovis/MinFeaturesForEstimate 10\
       --output "$RGBD_RESULTS_PATH/$d"\
       --output_name $OUTPUT\
       --quiet\
       $RGBD_ROOT_PATH/$d
done


#--Odom/Strategy 1\
#--Vis/CorType 1\


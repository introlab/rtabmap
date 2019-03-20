#!/bin/bash

PWD=$(pwd)
DOCKER=1 # set to 0 to use host rtabmap tool and relative datasets to this script

DATASET_ROOT_PATH='./datasets/euroc'
EUROC_RESULTS_PATH='./results/euroc'
mkdir -p $EUROC_RESULTS_PATH
if [ $DOCKER -eq 1 ]
then
    DATASET_ROOT_PATH='/root/datasets/euroc'
    EUROC_RESULTS_PATH='/root/results/euroc'
fi

LIST=( 'V1_01_easy' 'V1_02_medium' 'V1_03_difficult' 'V2_01_easy' 'V2_02_medium' 'V2_03_difficult' 'MH_01_easy' 'MH_02_easy' 'MH_03_medium' 'MH_04_difficult' 'MH_05_difficult')

STRATEGY=0
OUTPUT="test"
DISP=0
if [ $# -eq 4 ] 
then
    OUTPUT=$1
    STRATEGY=$2
    DISP=$3
    LIST=($4)
elif [ $# -eq 3 ] 
then
    OUTPUT=$1
    STRATEGY=$2
    DISP=$3
else
echo "Usage: run_euroc_datasets.sh \"output name\" \"odom strategy: 0=f2m 1=f2f 11=f2f_optflow 2=fovis 3=viso2 4=dvo 5=orbslam2 6=okvis(rect) 66=okvis(raw) 8=msckf_vio(rect) 88=msckf_vio(raw)\" \"Disparity: 0 or 1\" [sequence]"
exit
fi

F2F_params=""
if [ $STRATEGY -eq 11 ] # F2F optical flow
then
    STRATEGY=1
    F2F_params="--Vis/CorType 1 --Vis/MinInliers 20 --RGBD/OptimizeMaxError 0"
fi

if [ $STRATEGY -eq 1 ] # F2F
then
    F2F_params="--Odom/KeyFrameThr 0.6 $F2F_params"
fi

if [ $DISP -eq 1 ]
then
    F2F_params="--disp --StereoBM/NumDisparities 32 --StereoBM/UniquenessRatio 5 $F2F_params"
else
    F2F_params="--exposure_comp $F2F_params"
fi

if [ $STRATEGY -eq 66 ] # okvis raw images
then
    STRATEGY=6
    F2F_params="--raw --Odom/GuessMotion false --OdomOKVIS/ConfigPath ./config_fpga_p2_euroc.yaml $F2F_params"
fi
if [ $STRATEGY -eq 6 ] # okvis
then
    F2F_params="--Odom/GuessMotion false --OdomOKVIS/ConfigPath ./config_fpga_p2_euroc.yaml $F2F_params"
fi

if [ $STRATEGY -eq 88 ] # msckf_vio raw images
then
    STRATEGY=8
    F2F_params="--raw --Odom/GuessMotion false $F2F_params"
fi
if [ $STRATEGY -eq 8 ] # msckf_vio
then
    F2F_params="--Odom/GuessMotion false $F2F_params"
fi

if [ $STRATEGY -eq 99 ] # vins raw images
then
    STRATEGY=9
    F2F_params="--raw --Odom/GuessMotion false --OdomVINS/ConfigPath ./euroc_stereo_imu_config.yaml $F2F_params"
fi
if [ $STRATEGY -eq 9 ] # vins
then
    F2F_params="--Odom/GuessMotion false --OdomVINS/ConfigPath ./euroc_stereo_imu_config.yaml $F2F_params"
fi

if [ $STRATEGY -eq 100 ] # vins (stereo-only) raw images
then
    STRATEGY=9
    F2F_params="--raw --Odom/GuessMotion false --OdomVINS/ConfigPath ./euroc_stereo_config.yaml $F2F_params"
fi
if [ $STRATEGY -eq 10 ] # vins (stereo-only)
then
    F2F_params="--Odom/GuessMotion false --OdomVINS/ConfigPath ./euroc_stereo_config.yaml $F2F_params"
fi

RTABMAP_RGBD_TOOL="rtabmap-euroc_dataset"
if [ $DOCKER -eq 1 ]
then
    # Select rtabmap built with os2 support or not
    TOOL_PREFIX="/usr/local/bin"
    if [ $STRATEGY -eq 5 ]
    then
        TOOL_PREFIX="/root/rtabmap_os2/bin"
    elif [ $STRATEGY -eq 8 ]
    then
        TOOL_PREFIX="/root/rtabmap_msckf/bin"
    fi
    RTABMAP_RGBD_TOOL="docker run -v $PWD/datasets/euroc:$DATASET_ROOT_PATH -v $PWD/results/euroc:$EUROC_RESULTS_PATH -i -t --rm introlab3it/rtabmap:jfr2018 $TOOL_PREFIX/rtabmap-euroc_dataset"
    if [ $STRATEGY -eq 6 ]
    then
        xhost +
        RTABMAP_RGBD_TOOL="docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD/datasets/euroc:$DATASET_ROOT_PATH -v $PWD/results/euroc:$EUROC_RESULTS_PATH -i -t --rm introlab3it/rtabmap:jfr2018 $TOOL_PREFIX/rtabmap-euroc_dataset"
    fi
fi
echo $RTABMAP_RGBD_TOOL

for d in "${LIST[@]}"
do
    V203_params=""
    if [ "$d" == "V2_03_difficult" ]
    then
       if [[ $STRATEGY -eq 0 || $STRATEGY -eq 1 ]]
       then
           V203_params="--Vis/MinInliers 10 --Vis/PnPReprojError 3"
       fi
    fi

    taskset 0x1 $RTABMAP_RGBD_TOOL \
       --Rtabmap/PublishRAMUsage true\
       --RGBD/LinearUpdate 0\
       --RGBD/AngularUpdate 0\
       --RGBD/ProximityBySpace false\
       --Rtabmap/DetectionRate 2\
       --Kp/FlannRebalancingFactor 1.0\
       --Odom/Strategy $STRATEGY\
       --Mem/STMSize 30\
       --Mem/UseOdomFeatures false \
       --Mem/BinDataKept false \
       --Rtabmap/CreateIntermediateNodes false\
       $V203_params\
       $F2F_params\
       --OdomORBSLAM2/VocPath /root/ORBvoc.txt\
       --OdomORBSLAM2/ThDepth 35\
       --OdomORBSLAM2/MaxFeatures 1200\
       --output "$EUROC_RESULTS_PATH/$d"\
       --output_name $OUTPUT\
       $DATASET_ROOT_PATH/$d
done
# For V2_03_difficult: --Vis/MinInliers 10 --disp StereoBM/NumDisparities 32 --StereoBM/UniquenessRatio 5 --Vis/PnPReprojError 3



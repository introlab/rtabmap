#!/bin/bash

# docker pull introlab3it/rtabmap:jfr2018

mkdir -p results/kitti
./run_kitti_datasets.sh f2m 0 0
./run_kitti_datasets.sh f2f 11 0
./run_kitti_datasets.sh s2m 0 1
./run_kitti_datasets.sh s2s 1 1
./run_kitti_datasets.sh fovis 2 0
./run_kitti_datasets.sh viso2 3 0
./run_kitti_datasets.sh os2 5 0
./run_kitti_datasets.sh loam  7 1

mkdir -p results/tum
./run_tum_datasets.sh f2m 0
./run_tum_datasets.sh f2f 11
./run_tum_datasets.sh fovis 2
./run_tum_datasets.sh dvo 4
./run_tum_datasets.sh os2 5

mkdir -p results/euroc
./run_euroc_datasets.sh f2m 0 0
./run_euroc_datasets.sh f2f 11 0
./run_euroc_datasets.sh os2 5 0
./run_euroc_datasets.sh msckf 88 0
./run_euroc_datasets.sh fovis 2 0
./run_euroc_datasets.sh viso2 3 0
./run_euroc_datasets.sh okvis 66 0





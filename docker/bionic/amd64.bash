#!/bin/bash

# AliceVision v2.4.0 modified (Sept 13 2021)
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libsuitesparse-dev \
      libceres-dev \
      xorg-dev \
      libglu1-mesa-dev
git clone https://github.com/OpenImageIO/oiio.git
cd oiio && \
    git checkout Release-2.0.12 && \
    mkdir build && \
    cd build && \
    cmake -DUSE_PYTHON=OFF -DOIIO_BUILD_TESTS=OFF -DOIIO_BUILD_TOOLS=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r oiio
git clone https://github.com/assimp/assimp.git
cd assimp && \
    git checkout 71a87b653cd4b5671104fe49e2e38cf5dd4d8675 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r assimp
git clone https://github.com/alicevision/geogram.git
cd geogram && \
    git checkout v1.7.6 && \
    wget https://gist.githubusercontent.com/matlabbe/1df724465106c056ca4cc195c81d8cf0/raw/b3ed4cb8f9b270833a40d57d870a259eabfa4415/geogram_8b2ae61.patch && \
    git apply geogram_8b2ae61.patch && \
    ./configure.sh && \
    cd build/Linux64-gcc-dynamic-Release && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r geogram
git clone https://github.com/alicevision/AliceVision.git --recursive
cd AliceVision && \
    git checkout 0f6115b6af6183c524aa7fcf26141337c1cf3872 && \
    wget https://gist.githubusercontent.com/matlabbe/1df724465106c056ca4cc195c81d8cf0/raw/b3ed4cb8f9b270833a40d57d870a259eabfa4415/alicevision_0f6115b.patch && \
    git apply alicevision_0f6115b.patch && \
    mkdir build && \
    cd build && \
    ~/cmake -DALICEVISION_USE_CUDA=OFF -DALICEVISION_USE_APRILTAG=OFF -DALICEVISION_BUILD_SOFTWARE=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r AliceVision

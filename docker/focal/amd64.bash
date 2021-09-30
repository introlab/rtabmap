#!/bin/bash

# Azure Kinect DK
echo "Installing k4a..."
# Taken from https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190#issuecomment-822772494
# K4A binaries on 20.04 not released yet, we should take those from 18.04
echo "Download libk4a1.3_1.3.0_amd64.deb..."
curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb > /tmp/libk4a1.3_1.3.0_amd64.deb
echo "Download libk4a1.3-dev_1.3.0_amd64.deb..."
curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb > /tmp/libk4a1.3-dev_1.3.0_amd64.deb
echo "Download libk4abt1.0_1.0.0_amd64.deb..."
curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb > /tmp/libk4abt1.0_1.0.0_amd64.deb
echo "Download libk4abt1.0-dev_1.0.0_amd64.deb..."
curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb > /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
echo "Download k4a-tools_1.3.0_amd64.deb..."
curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb > /tmp/k4a-tools_1.3.0_amd64.deb
echo "Accept license..."
echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
dpkg -i /tmp/libk4a1.3_1.3.0_amd64.deb
dpkg -i /tmp/libk4a1.3-dev_1.3.0_amd64.deb
dpkg -i /tmp/libk4abt1.0_1.0.0_amd64.deb
dpkg -i /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
apt-get install -y libsoundio1
dpkg -i /tmp/k4a-tools_1.3.0_amd64.deb
rm /tmp/libk4a* /tmp/k4a*

# libfreenect2 
echo "Installing libfreenect2..."
apt-get install -y mesa-utils xserver-xorg-video-all libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
git clone https://github.com/OpenKinect/libfreenect2
cd libfreenect2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libfreenect2

# zed open capture
echo "Installing zed-open-capture..."
apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev wget
git clone https://github.com/stereolabs/zed-open-capture.git
cd zed-open-capture && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r zed-open-capture

# AliceVision v2.4.0 modified (Sept 13 2021)
echo "Installing AliceVision..."
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libsuitesparse-dev \
      libceres-dev \
      xorg-dev \
      libglu1-mesa-dev \
      wget
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
    cmake -DALICEVISION_USE_CUDA=OFF -DALICEVISION_USE_APRILTAG=OFF -DALICEVISION_BUILD_SOFTWARE=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r AliceVision

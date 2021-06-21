# Image: introlab3it/rtabmap:focal

FROM ros:noetic-perception

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-noetic-rtabmap-ros && \
    apt-get remove -y ros-noetic-rtabmap && \
    rm -rf /var/lib/apt/lists/

WORKDIR /root/

# GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 -y
RUN apt install libgtsam-dev libgtsam-unstable-dev -y

# libpointmatcher 
RUN git clone https://github.com/ethz-asl/libnabo.git
#commit February 13 2021
RUN cd libnabo && \
    git checkout 3cab7eed92bd5d4aed997347b8c8a2692a83a532 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libnabo
RUN git clone https://github.com/ethz-asl/libpointmatcher.git
#commit April 6 2021
RUN cd libpointmatcher && \
    git checkout 76f99fce0fe69e6384102a0343fdf8d262626e1f && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libpointmatcher
    
# libfreenect2 
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y mesa-utils xserver-xorg-video-all libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
RUN git clone https://github.com/OpenKinect/libfreenect2
RUN cd libfreenect2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libfreenect2
    
# RealSense2
RUN apt-get install -y ros-noetic-realsense2-camera

# Azure Kinect DK
# Taken from https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190#issuecomment-822772494
# K4A binaries on 20.04 not released yet, we should take those from 18.04
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb > /tmp/libk4a1.3_1.3.0_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb > /tmp/libk4a1.3-dev_1.3.0_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb > /tmp/libk4abt1.0_1.0.0_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb > /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb > /tmp/k4a-tools_1.3.0_amd64.deb
RUN echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN dpkg -i /tmp/libk4a1.3_1.3.0_amd64.deb
RUN dpkg -i /tmp/libk4a1.3-dev_1.3.0_amd64.deb
RUN dpkg -i /tmp/libk4abt1.0_1.0.0_amd64.deb
RUN dpkg -i /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
RUN apt-get install -y libsoundio1
RUN dpkg -i /tmp/k4a-tools_1.3.0_amd64.deb
RUN rm /tmp/libk4a* /tmp/k4a*

# zed open capture
RUN apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev wget
RUN git clone https://github.com/stereolabs/zed-open-capture.git
RUN cd zed-open-capture && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r zed-open-capture

# AliceVision
# Issue: It could be possible to use version >2.2, but there is a seg fault after texturing the mesh (see #564).
RUN apt-get update && apt-get install -y \
      libsuitesparse-dev \
      libceres-dev \
      xorg-dev \
      libglu1-mesa-dev \
      wget \
      python-is-python3
RUN git clone https://github.com/OpenImageIO/oiio.git
RUN cd oiio && \
    git checkout Release-2.0.12 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r oiio
RUN git clone https://github.com/alembic/alembic.git
RUN cd alembic && \
    git checkout 1.7.12 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r alembic
RUN git clone -b v1.7.1 https://github.com/alicevision/geogram.git
RUN cd geogram && \
    ./configure.sh && \
    cd build/Linux64-gcc-dynamic-Release && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r geogram
RUN git clone -b v2.2.0 https://github.com/alicevision/AliceVision.git --recursive
RUN cd AliceVision && \
    wget https://gist.githubusercontent.com/matlabbe/469bba5e7733ad6f2e3d7857b84f1f9e/raw/f0545b36028a1156e857ed433547fdade0cbf53f/alice_vision_eigen.patch && \
    git apply alice_vision_eigen.patch && \
    mkdir build && \
    cd build && \
    cmake -DALICEVISION_USE_CUDA=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r AliceVision
    
# PDAL
RUN apt-get install -y libpdal-dev

# Clone source code
ARG CACHE_DATE=2016-01-01
RUN git clone https://github.com/introlab/rtabmap.git

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DWITH_ALICE_VISION=ON .. && \
    make -j2 && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


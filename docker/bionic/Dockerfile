# Image: introlab3it/rtabmap:bionic

FROM ros:melodic-perception

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-melodic-rtabmap-ros && \
    apt-get remove -y ros-melodic-rtabmap && \
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

# AliceVision
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libsuitesparse-dev \
      libceres-dev \
      xorg-dev \
      libglu1-mesa-dev \
      wget
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
RUN git clone https://github.com/alicevision/geogram.git
RUN cd geogram && \
    git checkout v1.7.1 && \
    ./configure.sh && \
    cd build/Linux64-gcc-dynamic-Release && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r geogram
RUN git clone https://github.com/alicevision/AliceVision.git --recursive
RUN cd AliceVision && \
    git checkout v2.2.0 && \
    wget https://gist.githubusercontent.com/matlabbe/469bba5e7733ad6f2e3d7857b84f1f9e/raw/edaa88ed38344219af1cc919a5597f5a74445336/alice_vision_eigen.patch && \
    git apply alice_vision_eigen.patch && \
    mkdir build && \
    cd build && \
    cmake -DALICEVISION_USE_CUDA=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r AliceVision

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


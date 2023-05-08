# Image: introlab3it/rtabmap:bionic

FROM ros:melodic-perception

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-melodic-rtabmap-ros && \
    apt-get remove -y ros-melodic-rtabmap* && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

# GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 -y
RUN apt-get update && apt install libgtsam-dev libgtsam-unstable-dev -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# MRPT
RUN add-apt-repository ppa:joseluisblancoc/mrpt-stable -y
RUN apt-get update && apt install libmrpt-poses-dev -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/

ARG TARGETPLATFORM
ENV TARGETPLATFORM=${TARGETPLATFORM:-linux/amd64}
RUN echo "I am building for $TARGETPLATFORM"

# arm64
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then ln -s /usr/bin/cmake ~/cmake; fi

# cmake >=3.11 required for amd64 dependencies
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then apt update && apt install -y wget && apt-get clean && rm -rf /var/lib/apt/lists/ && \
    wget -nv https://github.com/Kitware/CMake/releases/download/v3.17.0/cmake-3.17.0-Linux-x86_64.tar.gz && \
    tar -xzf cmake-3.17.0-Linux-x86_64.tar.gz && \
    rm cmake-3.17.0-Linux-x86_64.tar.gz &&\
    ln -s ~/cmake-3.17.0-Linux-x86_64/bin/cmake ~/cmake; fi
    
#commit Aug 6 2020
RUN apt-get update && apt install wget && apt-get clean && rm -rf /var/lib/apt/lists/
RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch && \
    mkdir build && \
    cd build && \
    ~/cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r opengv

# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Copy current source code
COPY . /root/rtabmap

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    ~/cmake -DWITH_OPENGV=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


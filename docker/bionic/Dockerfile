# Image: introlab3it/rtabmap:bionic

FROM ubuntu:18.04

# Install build dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      git \
      cmake \
      libopencv-dev \
      libproj-dev \
      libqt5svg5-dev \
      libfreenect-dev \
      libopenni2-dev \
      ffmpeg \
      software-properties-common

WORKDIR /root/

# Clone source code
RUN git clone https://github.com/introlab/rtabmap.git

# Build RTAB-Map project
RUN cd rtabmap/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

WORKDIR /root
    

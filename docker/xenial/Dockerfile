# Image: introlab3it/rtabmap:xenial

FROM ubuntu:16.04

# Install build dependencies
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      libopencv-dev \
      git \
      cmake \
      libproj-dev \
      libqt5svg5-dev \
      software-properties-common

# Clone source code
WORKDIR /root/
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
    

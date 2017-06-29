# Image: introlab3it/rtabmap:zesty

FROM ubuntu:17.04

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

# Issue: http://www.pcl-users.org/Build-failure-on-Ubuntu-17-04-td4044552.html
RUN sed -i 's|/usr/lib/libmpi.so;||g' /usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake

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
    

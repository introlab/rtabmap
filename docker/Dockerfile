FROM ubuntu:16.04

# Install build dependencies
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      libopencv-dev \
      git \
      cmake \
      libproj-dev \
      libqt5svg5-dev

# Clone source code
WORKDIR /root/
RUN git clone https://github.com/introlab/rtabmap.git

# Build RTAB-Map project
WORKDIR /root/rtabmap
RUN cd build && \
    cmake .. && \
    make -j$(nproc)

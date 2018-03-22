# Image: introlab3it/rtabmap:artful

FROM ubuntu:17.10

# Install build dependencies
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      git \
      cmake \
      libproj-dev \
      libqt5svg5-dev \
      libfreenect-dev \
      libopenni2-dev \
      ffmpeg \
      software-properties-common

WORKDIR /root/

# using OpenCV from source because ffmpeg (libavutil..) error with opencv binaries:
# rtabmap: symbol lookup error: /usr/lib/x86_64-linux-gnu/libavutil.so.55: undefined symbol: vdp_device_create_x11
# ffmpeg above should be installed before building opencv

RUN git clone https://github.com/opencv/opencv_contrib.git
RUN git clone https://github.com/opencv/opencv.git
RUN cd opencv && mkdir build && cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf opencv opencv_contrib

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
    

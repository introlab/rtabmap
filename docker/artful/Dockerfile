# Image: introlab3it/rtabmap:artful

FROM ubuntu:17.10

# Install build dependencies
RUN apt-get update && apt-get install -y \
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

# GTSAM
RUN git clone https://bitbucket.org/gtborg/gtsam.git
RUN cd gtsam && \
    git checkout 4.0.0-alpha2 && \
    mkdir build && \
    cd build && \
    cmake -DMETIS_SHARED=ON -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r gtsam

# g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && \
    git checkout 20170730_git && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_LGPL_SHARED_LIBS=ON -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r g2o

# libpointmatcher
RUN git clone https://github.com/ethz-asl/libnabo.git
RUN cd libnabo && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libnabo
RUN git clone https://github.com/ethz-asl/libpointmatcher.git
RUN cd libpointmatcher && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libpointmatcher

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
    

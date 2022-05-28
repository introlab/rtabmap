# Image: introlab3it/rtabmap:xenial

FROM ros:kinetic-perception

# Install/build dependencies
RUN apt-get update && \
    apt-get install -y ros-kinetic-rtabmap-ros && \
    apt-get remove -y ros-kinetic-rtabmap && \
    rm -rf /var/lib/apt/lists/

WORKDIR /root/

# GTSAM
RUN git clone https://bitbucket.org/gtborg/gtsam.git
RUN cd gtsam && \
    git checkout 4.0.0-alpha2 && \
    mkdir build && \
    cd build && \
    cmake -DMETIS_SHARED=ON -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r gtsam

# libpointmatcher 
RUN git clone https://github.com/ethz-asl/libnabo.git
#commit Apr 25 2018
RUN cd libnabo && \
    git checkout 7e378f6765393462357b8b74d8dc8c5554542ae6 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libnabo
RUN git clone https://github.com/ethz-asl/libpointmatcher.git
#commit Jan 19 2018
RUN cd libpointmatcher && \
    git checkout 00004bd41e44a1cf8de24ad87e4914760717cbcc && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libpointmatcher

# Copy current source code
COPY . /root/rtabmap

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig
    

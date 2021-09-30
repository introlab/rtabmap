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
    
# PDAL
RUN apt-get install -y libpdal-dev

# RealSense2
RUN apt-get install -y ros-noetic-librealsense2

ARG TARGETPLATFORM
ENV TARGETPLATFORM=${TARGETPLATFORM:-linux/amd64}
RUN echo "I am building for $TARGETPLATFORM"

ENV DEBIAN_FRONTEND=noninteractive

# amd64-only dependencies
ADD amd64.bash /root/amd64.bash
RUN chmod +x amd64.bash
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then /bin/bash -c "./amd64.bash"; fi

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


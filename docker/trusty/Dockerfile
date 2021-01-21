# Image: introlab3it/rtabmap:trusty

FROM ros:indigo-perception

# Install/build dependencies
RUN apt-get update && \
    apt-get install -y ros-indigo-rtabmap-ros && \
    apt-get remove -y ros-indigo-rtabmap && \
    rm -rf /var/lib/apt/lists/

WORKDIR /root/

# Clone source code
ARG CACHE_DATE=2016-01-01
RUN git clone https://github.com/introlab/rtabmap.git

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
    

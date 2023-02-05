# Image: introlab3it/rtabmap:jammy

FROM osrf/ros:humble-desktop

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-humble-rtabmap-ros && \
    apt-get remove -y ros-humble-rtabmap && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/
   
# PDAL
RUN apt-get update && apt-get install -y libpdal-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# RealSense2
RUN apt-get update && apt-get install -y ros-humble-librealsense2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/

ARG TARGETPLATFORM
ENV TARGETPLATFORM=${TARGETPLATFORM:-linux/amd64}
RUN echo "I am building for $TARGETPLATFORM"

ENV DEBIAN_FRONTEND=noninteractive

# libfreenect2 
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then echo "Installing libfreenect2..." && \
    apt-get update && apt-get install -y mesa-utils xserver-xorg-video-all libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    git clone https://github.com/OpenKinect/libfreenect2 && \
    cd libfreenect2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libfreenect2; fi

# zed open capture
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then echo "Installing zed-open-capture..." && \
    apt-get update && apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev wget && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    git clone https://github.com/stereolabs/zed-open-capture.git && \
    cd zed-open-capture && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r zed-open-capture; fi
    
#commit Aug 6 2020
RUN apt-get update && apt install wget && \
    apt-get clean && rm -rf /var/lib/apt/lists/
RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r opengv

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Copy current source code
COPY . /root/rtabmap

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib/aarch64-linux-gnu

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DWITH_OPENGV=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


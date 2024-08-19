# Image: introlab3it/rtabmap:noble-deps

FROM ubuntu:24.04

ARG TARGETPLATFORM
ENV TARGETPLATFORM=${TARGETPLATFORM:-linux/amd64}
RUN echo "I am building for $TARGETPLATFORM"

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
RUN apt update && \
    apt install software-properties-common -y && \
    add-apt-repository universe && \
    apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# Install build dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt-get install -y \
      git \
      wget \
      libtbb-dev \
      libproj-dev \
      libpcl-dev \
      liboctomap-dev \
      libfreenect-dev \
      ros-jazzy-ros-base \
      ros-dev-tools \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-geometry \
      ros-jazzy-laser-geometry \
      ros-jazzy-pcl-conversions \
      ros-jazzy-rviz-common \
      ros-jazzy-rviz-rendering \
      ros-jazzy-rviz-default-plugins \
      ros-jazzy-pcl-ros \
      ros-jazzy-imu-filter-madgwick \
      ros-jazzy-velodyne \
      ros-jazzy-image-transport \
      ros-jazzy-octomap-msgs \
      ros-jazzy-libg2o \
      ros-jazzy-libpointmatcher \
      ros-jazzy-qt-gui-cpp \
      ros-jazzy-diagnostic-updater && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

# GTSAM (we cannot use ros-jazzy-gtsam on Noble because it is linked to a different internal Eigen version than system one)
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2.0 && \ 
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r gtsam

# libfreenect2 
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then echo "Installing libfreenect2..." && \
    apt-get update && apt-get install -y mesa-utils xserver-xorg-video-all libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    git clone https://github.com/OpenKinect/libfreenect2 && \
    cd libfreenect2 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r libfreenect2; fi

# zed open capture
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then echo "Installing zed-open-capture..." && \
    apt-get update && apt install -y libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev wget && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    git clone https://github.com/stereolabs/zed-open-capture.git && \
    cd zed-open-capture && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd && \
    rm -r zed-open-capture; fi

# OpenCV with all modules (same version than distro version to avoid conflicts with cv_bridge ros package)
COPY ./docker/noble/deps/opencv_4_6_0_ffmpeg.patch .
RUN git clone --branch 4.6.0 https://github.com/opencv/opencv.git && \
    git clone --branch 4.6.0 https://github.com/opencv/opencv_contrib.git && \
    mv opencv_4_6_0_ffmpeg.patch opencv/. && \
    cd opencv && \
    git apply opencv_4_6_0_ffmpeg.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DWITH_TBB=ON -DWITH_ADE=OFF -DWITH_OPENMP=ON -DBUILD_opencv_python3=OFF -DBUILD_opencv_python_bindings_generator=OFF -DBUILD_opencv_python_tests=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf opencv opencv_contrib

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

COPY ./docker/noble/deps/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib/aarch64-linux-gnu

# for jetson (https://github.com/introlab/rtabmap/issues/776)
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra


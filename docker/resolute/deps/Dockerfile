
# Image: introlab3it/rtabmap:resolute-deps

FROM ubuntu:26.04

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
      libceres-dev \
      ros-lyrical-ros-base \
      ros-dev-tools \
      ros-lyrical-cv-bridge \
      ros-lyrical-image-geometry \
      ros-lyrical-laser-geometry \
      ros-lyrical-pcl-conversions \
      ros-lyrical-rviz-common \
      ros-lyrical-rviz-rendering \
      ros-lyrical-rviz-default-plugins \
      ros-lyrical-pcl-ros \
      ros-lyrical-imu-filter-madgwick \
      ros-lyrical-image-transport \
      ros-lyrical-octomap-msgs \
      ros-lyrical-libg2o \
      ros-lyrical-gtsam \
      ros-lyrical-qt-gui-cpp \
      ros-lyrical-diagnostic-updater && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

# libfreenect2 
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then echo "Installing libfreenect2..." && \
    apt-get update && apt-get install -y mesa-utils xserver-xorg-video-all libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    git clone https://github.com/OpenKinect/libfreenect2 && \
    cd libfreenect2 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. && \
    make -j4 && \
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
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. && \
    make -j4 && \
    make install && \
    cd && \
    rm -r zed-open-capture; fi

# OpenCV with all modules (same version than distro version to avoid conflicts with cv_bridge ros package)
RUN git clone --branch 4.10.0 https://github.com/opencv/opencv.git && \
    git clone --branch 4.10.0 https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    # FFmpeg 7/8 compatibility (Ubuntu 26.04): avcodec_close / av_stream_get_side_data removed
    git -c user.email=docker@build -c user.name=docker cherry-pick -x 90c444abd387ffa70b2e72a34922903a2f0f4f5a 443d0ae63fad6dfd8c485d609203db16c8bd0ec3 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DWITH_TBB=ON -DWITH_ADE=OFF -DWITH_OPENMP=ON -DBUILD_opencv_python3=OFF -DBUILD_opencv_python_bindings_generator=OFF -DBUILD_opencv_python_tests=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -rf opencv opencv_contrib

RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. && \
    make -j4 && \
    make install && \
    cd && \
    rm -r opengv

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN echo -e '#!/bin/bash\nset -e\n\n# setup ros2 environment\nsource "/opt/ros/lyrical/setup.bash" --\nexec "$@"' > /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/lyrical/lib/x86_64-linux-gnu:/opt/ros/lyrical/lib/aarch64-linux-gnu

# for jetson (https://github.com/introlab/rtabmap/issues/776)
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra


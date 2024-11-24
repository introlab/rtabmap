
# humble or jazzy
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-${ROS_DISTRO}-rtabmap-ros libqt6* qt6* qml6* && \
    apt-get remove -y ros-${ROS_DISTRO}-rtabmap* ros-${ROS_DISTRO}-gtsam ros-${ROS_DISTRO}-libg2o libpcl* libqt5* qt5* libvtk* libopencv* && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Build latest VTK with Qt6
RUN git clone https://github.com/Kitware/VTK.git && \
    cd VTK && \
    mkdir build && \
    cd build && \
    cmake -DVTK_GROUP_ENABLE_Qt=YES .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf VTK
    
# Build latest PCL with latest VTK
# Make sure all libraries depending on Eigen are built with same CXX standard (17)
RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DBUILD_tools=OFF -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_MARCHNATIVE=OFF -DPCL_ENABLE_SSE=OFF .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf pcl
    
# Build latest OpenCV
RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_opencv_python3=OFF -DBUILD_opencv_python_bindings_generator=OFF -DBUILD_opencv_python_tests=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf opencv opencv_contrib

# Build latest gtsam
RUN git clone https://github.com/borglab/gtsam.git && \
 cd gtsam && \
 mkdir build && \
 cd build && \
 cmake -DCMAKE_CXX_STANDARD=17 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_INSTALL_CPPUNILITE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
 cmake --build . --config Release --target install && \
 cd ../.. && \
 rm -rf gtsam

# Build latest g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git  && \
 cd g2o && \
 mkdir build && \
 cd build && \
 cmake -DCMAKE_CXX_STANDARD=17 -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
 cmake --build . --config Release --target install && \
 cd ../.. && \
 rm -rf g2o

RUN mkdir -p /root/Documents/RTAB-Map

# Copy current source code
COPY . /root/rtabmap

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig



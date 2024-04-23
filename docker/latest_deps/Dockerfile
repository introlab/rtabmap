
FROM osrf/ros:humble-desktop

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-humble-rtabmap-ros libqt6* qt6* && \
    apt-get remove -y ros-humble-rtabmap* ros-humble-libg2o libpcl* libqt5* qt5* libvtk* libopencv* && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib/x86_64-linux-gnu

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
RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_tools=OFF .. && \
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
    rm -rf opencv

# Build latest gtsam
RUN git clone https://github.com/borglab/gtsam.git && \
 cd gtsam && \
 mkdir build && \
 cd build && \
 cmake -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_INSTALL_CPPUNILITE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
 cmake --build . --config Release --target install && \
 cd ../.. && \
 rm -rf gtsam

# Build latest g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git  && \
 cd g2o && \
 mkdir build && \
 cd build && \
 cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
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



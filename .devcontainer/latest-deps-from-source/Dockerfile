ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop

# Install build dependencies with Qt6 (issue: rtabmap has black window)
#RUN apt-get update && \
#    apt-get install -y git software-properties-common ros-${ROS_DISTRO}-rtabmap-ros libqt6* qt6* qml6* && \
#    apt-get remove -y ros-${ROS_DISTRO}-rtabmap* ros-${ROS_DISTRO}-gtsam ros-${ROS_DISTRO}-libg2o libpcl* libqt5* qt5* libvtk* libopencv* && \
#    apt-get clean && rm -rf /var/lib/apt/lists/

# Install build dependencies with Qt5
RUN apt-get update && \
    apt-get install -y git software-properties-common ros-${ROS_DISTRO}-rtabmap-ros && \
    apt-get remove -y ros-${ROS_DISTRO}-rtabmap* ros-${ROS_DISTRO}-gtsam ros-${ROS_DISTRO}-libg2o libpcl* libvtk* libopencv* && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# remove ubuntu user
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

RUN apt-get update && apt-get install -y sudo && \
    apt-get clean && rm -rf /var/lib/apt/lists/

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -ex && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -a -G sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /home/${USERNAME}/Documents/RTAB-Map

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# We build main dependencies from source, cleaning up the
# build directory after install but keeping the source code
# to change version/reinstall from inside the dev container
# if needed.

# Build latest VTK with Qt6
RUN git clone https://github.com/Kitware/VTK.git
RUN cd VTK && \
    mkdir build && \
    cd build && \
    cmake -DVTK_GROUP_ENABLE_Qt=YES .. && \
    make -j$(nproc) && \
    make install && \
    make clean
    
# Build latest PCL with latest VTK
# Make sure all libraries depending on Eigen are built with same CXX standard (17)
RUN git clone https://github.com/PointCloudLibrary/pcl.git
RUN cd pcl && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DBUILD_tools=ON -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_MARCHNATIVE=OFF -DPCL_ENABLE_SSE=OFF .. && \
    make -j$(nproc) && \
    make install && \
    make clean
    
# Build latest OpenCV
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_opencv_python3=OFF -DBUILD_opencv_python_bindings_generator=OFF -DBUILD_opencv_python_tests=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules .. && \
    make -j$(nproc) && \
    make install && \
    make clean

# Build latest gtsam
RUN git clone https://github.com/borglab/gtsam.git
RUN cd gtsam && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_INSTALL_CPPUNILITE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    make clean

# Build latest g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    make clean

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu

RUN ldconfig

RUN chown -R ${USERNAME} /home/${USERNAME}

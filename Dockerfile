# FROM mitmrg/gtsam-opencv-pcl:latest
# Possibly useful unsure
FROM ros:melodic-perception

# ENV GIT_SSH_COMMAND 'ssh -i ~/.ssh/id_rsa -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no'
# ARG USER_ID
# RUN adduser --uid $USER_ID mrg --disabled-password --gecos="mrg"
# RUN usermod -aG sudo mrg
# RUN echo "mrg ALL=NOPASSWD: ALL" >> /etc/sudoers
# USER mrg
# WORKDIR /home/mrg


USER root

# Install build dependencies
RUN apt-get update && \
    apt-get install -y ros-melodic-rtabmap ros-melodic-rtabmap-ros && \
    apt-get remove -y ros-melodic-rtabmap ros-melodic-rtabmap-ros

# g2o from source
RUN apt-get remove -y ros-melodic-libg2o && \
    git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    mkdir build && cd build && \
    cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
    make -j4 && \
    make install

# GTSAM
RUN cd ~ && \
    git clone --branch 4.0.0 https://github.com/borglab/gtsam.git && \
    cd ~/gtsam/ && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_UNSTABLE=ON .. && \
    make check -j4 && \
    make install -j4

# RUN cd ~ && \
#     git clone https://github.com/MarineRoboticsGroup/rtabmap.git rtabmap && \
#     cd ~/rtabmap && \
#     git checkout master && \
#     cd ~/rtabmap/build && \
#     cmake .. && \
#     make -j$(nproc) && \
#     make install


# RUN git clone https://bitbucket.org/gtborg/gtsam.git
# RUN cd gtsam && \
#     git checkout 4.0.0-alpha2 && \
#     mkdir build && \
#     cd build && \
#     cmake -DMETIS_SHARED=ON -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r gtsam

# libpointmatcher
# RUN git clone https://github.com/ethz-asl/libnabo.git
# #commit Apr 25 2018
# RUN cd libnabo && \
#     git checkout 7e378f6765393462357b8b74d8dc8c5554542ae6 && \
#     mkdir build && \
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r libnabo
# RUN git clone https://github.com/ethz-asl/libpointmatcher.git
# #commit Jan 19 2018
# RUN cd libpointmatcher && \
#     git checkout 00004bd41e44a1cf8de24ad87e4914760717cbcc && \
#     mkdir build && \
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r libpointmatcher

# # AliceVision
# RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
#       libsuitesparse-dev \
#       libceres-dev \
#       xorg-dev \
#       libglu1-mesa-dev \
#       wget
# RUN git clone https://github.com/OpenImageIO/oiio.git
# RUN cd oiio && \
#     git checkout Release-2.0.12 && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r oiio
# RUN git clone https://github.com/alembic/alembic.git
# RUN cd alembic && \
#     git checkout 1.7.12 && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r alembic
# RUN git clone https://github.com/alicevision/geogram.git
# RUN cd geogram && \
#     git checkout v1.7.1 && \
#     ./configure.sh && \
#     cd build/Linux64-gcc-dynamic-Release && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r geogram
# RUN git clone https://github.com/alicevision/AliceVision.git --recursive
# RUN cd AliceVision && \
#     git checkout v2.2.0 && \
#     wget https://gist.githubusercontent.com/matlabbe/469bba5e7733ad6f2e3d7857b84f1f9e/raw/edaa88ed38344219af1cc919a5597f5a74445336/alice_vision_eigen.patch && \
#     git apply alice_vision_eigen.patch && \
#     mkdir build && \
#     cd build && \
#     cmake -DALICEVISION_USE_CUDA=OFF .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd && \
#     rm -r AliceVision

# # Clone source code
# ARG CACHE_DATE=2016-01-01
# RUN git clone https://github.com/introlab/rtabmap.git

# RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# # Build RTAB-Map project
# RUN source /ros_entrypoint.sh && \
#     cd rtabmap/build && \
#     cmake .. && \
#     make && \
#     make install && \
#     cd ../.. && \
#     rm -rf rtabmap && \
#     ldconfig

# cd rtabmap
# docker build -t rtabmap_frontiers -f docker/frontiers2022/Dockerfile .
FROM nvcr.io/nvidia/pytorch:22.08-py3

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      git \
      cmake \
      libyaml-cpp-dev \
      software-properties-common \
      pkg-config \ 
      wget \
      libsuitesparse-dev

# g2o (removing config because cs.h not found error: https://github.com/introlab/rtabmap/issues/835)
RUN git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    git checkout 20201223_git && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_LGPL_SHARED_LIBS=ON -DG2O_BUILD_APPS=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -rf g2o /usr/local/lib/cmake/g2o

# GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install -y libgtsam-dev libgtsam-unstable-dev


# OpenCV with xfeatures2d and nonfree modules
RUN git clone https://github.com/opencv/opencv_contrib.git && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv_contrib && \
    git checkout tags/4.2.0 && \
    cd /workspace && \
    cd opencv && \
    git checkout tags/4.2.0 && \
    mkdir build && \
    cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/workspace/opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -rf opencv opencv_contrib

# Copy current source code
COPY . rtabmap

RUN cp -r rtabmap/archive/2022-IlluminationInvariant/scripts scripts && \
    chmod +x scripts/*.sh 

# Superpoint model
RUN cd scripts && \
    wget https://github.com/magicleap/SuperPointPretrainedNetwork/raw/master/superpoint_v1.pth && \
    wget https://raw.githubusercontent.com/magicleap/SuperPointPretrainedNetwork/master/demo_superpoint.py && \
    python trace.py && \
    rm superpoint_v1.pth demo_superpoint.py
    
# Superglue
RUN cd scripts && \
    git clone https://github.com/magicleap/SuperGluePretrainedNetwork && \
    cp /workspace/rtabmap/corelib/src/python/rtabmap_superglue.py SuperGluePretrainedNetwork/.

# Build RTAB-Map
RUN cd rtabmap && \
    cd build && \
    cmake -DTorch_DIR=/opt/conda/lib/python3.8/site-packages/torch/share/cmake/Torch -DWITH_TORCH=ON -DWITH_PYTHON=ON -DWITH_QT=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -rf rtabmap



#!/bin/bash

# Setup java 1.8
apt-get update
apt-get install -y --no-install-recommends apt-utils
apt-get install -y unzip wget ant
apt-get install -y default-jre default-jdk

# Setup android sdk
echo "wget android-sdk..."
wget -nv https://dl.google.com/android/repository/tools_r25.2.3-linux.zip
unzip -qq tools_r25.2.3-linux.zip
rm tools_r25.2.3-linux.zip
mkdir $ANDROID_HOME
mv tools $ANDROID_HOME/.
#android list sdk --all --extended
echo y | android update sdk --no-ui --all --filter platform-tools,android-19,build-tools-19.1.0

# Setup android ndk
echo "wget android-ndk..."
wget -nv https://dl.google.com/android/repository/android-ndk-r14-linux-x86_64.zip
unzip -qq android-ndk-r14-linux-x86_64.zip
rm android-ndk-r14-linux-x86_64.zip
mv android-ndk-r14 /opt/.

# Install directory for all dependencies
mkdir -p /opt/android/share
wget -nv https://github.com/introlab/rtabmap/raw/master/cmake_modules/android.toolchain.cmake
mv android.toolchain.cmake /opt/android/share/.

# Boost
echo "wget boost..."
wget -nv https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz
tar -xzf boost_1_59_0.tar.gz
cd boost_1_59_0
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/489ff2869eccd6f8d03ffb9090ef839108762741/BoostConfig.cmake.in
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/e7fbf0e301cfea417a7aa69989a761a4de08b8c3/CMakeLists.txt
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make
make install
cd
rm -r boost_1_59_0.tar.gz boost_1_59_0

# eigen
echo "wget eigen..."
wget -nv http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz
tar -xzf 3.2.7.tar.gz
cd eigen-eigen-b30b87236a1b
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make
make install
cd
rm -r 3.2.7.tar.gz eigen-eigen-b30b87236a1b

# FLANN
echo "wget flann..."
wget -nv http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
unzip -qq flann-1.8.4-src.zip
cd flann-1.8.4-src
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make 
make install
cd
rm -r flann-1.8.4-src.zip flann-1.8.4-src

# GTSAM
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout fbb9d3bdda8b88df51896bc401bfd170573e66f5
# patch
wget -nv https://gist.github.com/matlabbe/726b490c658afd3293f4b3f2f501b863/raw/df09fc8e238a495d66b062d92dc1c1fb20a581e8/gtsam_GKlib_android_fix.patch
git apply gtsam_GKlib_android_fix.patch
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make 
make install
cd
rm -r gtsam

# g2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make
make install
cd
rm -r g2o


#CMake 3.7 for VTK
echo "wget cmake3.7..."
wget -nv https://cmake.org/files/v3.7/cmake-3.7.2-Linux-x86_64.tar.gz
tar -xzf cmake-3.7.2-Linux-x86_64.tar.gz
rm cmake-3.7.2-Linux-x86_64.tar.gz

# VTK
git clone https://github.com/Kitware/VTK.git
cd VTK
git checkout tags/v7.1.1
mkdir build
cd build
~/cmake-3.7.2-Linux-x86_64/bin/cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_ANDROID_BUILD=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make
cp -r CMakeExternals/Install/vtk-android/* /opt/android/.
cd
rm -r VTK cmake-3.7.2-Linux-x86_64

# PCL
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout tags/pcl-1.8.0
# patch
wget -nv https://gist.github.com/matlabbe/41812e50e459b2f27b331a2343569e5d/raw/b97cf6dda9aa9487ee75a3cc4e8826b216a1ad06/pcl_1_8_0_vtk_android_support.patch
git apply pcl_1_8_0_vtk_android_support.patch
mkdir build
cd build
# do it 2 times because there is a cmake error on the first time and not the second time!?
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
cmake -DCMAKE_TOOLCHAIN_FILE=/opt/android/share/android.toolchain.cmake -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android ..
make
make install
cd
rm -r pcl

# OpenCV
echo "wget opencv..."
wget -nv https://downloads.sourceforge.net/project/opencvlibrary/opencv-android/3.2.0/opencv-3.2.0-android-sdk.zip
unzip -qq opencv-3.2.0-android-sdk.zip
rm opencv-3.2.0-android-sdk.zip
mv OpenCV-android-sdk /opt/.

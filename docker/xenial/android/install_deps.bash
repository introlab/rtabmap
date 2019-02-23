#!/bin/bash

if [ $# -ne 1 ]; then
    echo "install_deps.bash android_install_prefix   # Example: install_deps.bash /opt/android"
    exit 1
fi

cpus=-j4
prefix=$1
pwd=$(pwd)
wget -nv https://github.com/introlab/rtabmap/raw/master/cmake_modules/android.toolchain.cmake

# Install directory for all dependencies
mkdir -p $prefix/armeabi-v7a
mkdir -p $prefix/arm64-v8a

# Boost
echo "wget boost..."
wget -nv https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz
tar -xzf boost_1_59_0.tar.gz
cd boost_1_59_0
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/489ff2869eccd6f8d03ffb9090ef839108762741/BoostConfig.cmake.in
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/e7fbf0e301cfea417a7aa69989a761a4de08b8c3/CMakeLists.txt
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -r boost_1_59_0.tar.gz boost_1_59_0

# eigen
echo "wget eigen..."
wget -nv http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz
tar -xzf 3.2.7.tar.gz
cd eigen-eigen-b30b87236a1b
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -r 3.2.7.tar.gz eigen-eigen-b30b87236a1b

# FLANN
echo "wget flann..."
wget -nv http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
unzip -qq flann-1.8.4-src.zip
cd flann-1.8.4-src
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
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
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -rf gtsam

# g2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout a3f7706bdbb849b2808dc3e1b7aee189f63b498e
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -rf g2o

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
$pwd/cmake-3.7.2-Linux-x86_64/bin/cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_ANDROID_BUILD=ON -DANDROID_ARCH_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
cp -r CMakeExternals/Install/vtk-android/* $prefix/armeabi-v7a/.
rm -r *
$pwd/cmake-3.7.2-Linux-x86_64/bin/cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_ANDROID_BUILD=ON -DANDROID_ARCH_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
cp -r CMakeExternals/Install/vtk-android/* $prefix/arm64-v8a/.
cd $pwd
rm -rf VTK cmake-3.7.2-Linux-x86_64

# PCL
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout tags/pcl-1.8.0
# patch
wget -nv https://gist.github.com/matlabbe/41812e50e459b2f27b331a2343569e5d/raw/921bbb90d5115f7aec1640f255aec3036a518553/pcl_1_8_0_vtk_android_support.patch
git apply pcl_1_8_0_vtk_android_support.patch
mkdir build
cd build
# do it 2 times because there is a cmake error on the first time and not the second time!?
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
# do it 2 times because there is a cmake error on the first time and not the second time!?
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -rf pcl

# OpenCV
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout tags/3.4.2
cd $pwd
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout tags/3.4.2
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DOPENCV_EXTRA_MODULES_PATH=$pwd/opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ..
make $cpus
make install
rm -r *
cmake -DCMAKE_TOOLCHAIN_FILE=$pwd/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DOPENCV_EXTRA_MODULES_PATH=$pwd/opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ..
make $cpus
make install
cd $pwd
rm -rf opencv opencv_contrib

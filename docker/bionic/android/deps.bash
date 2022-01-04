#!/bin/bash

if [ $# -ne 1 ]; then
    echo "deps.bash android_install_prefix   # Example: setup.bash /opt/android"
    exit 1
fi

cpus=-j12
prefix=$1
pwd=$(pwd)
ANDROID_NATIVE_API_LEVEL=23

# Download all dependencies
echo "Downloading cmake... (1/10)"
wget -nv https://github.com/Kitware/CMake/releases/download/v3.17.0/cmake-3.17.0-Linux-x86_64.tar.gz
echo "Downloading boost... (2/10)"
wget -nv https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz
echo "Downloading eigen... (3/10)"
curl -L https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o 3.3.9.tar.gz
echo "Downloading flann... (4/10)"
git clone -b 1.8.4 https://github.com/mariusmuja/flann.git
echo "Downloading gtsam... (5/10)"
git clone https://bitbucket.org/gtborg/gtsam.git
echo "Downloading g2o... (6/10)"
git clone https://github.com/RainerKuemmerle/g2o.git
echo "Downloading VTK... (7/10)"
git clone https://github.com/Kitware/VTK.git
echo "Downloading pcl... (8/10)"
git clone https://github.com/PointCloudLibrary/pcl.git
echo "Downloading opencv_contrib... (9/10)"
git clone https://github.com/opencv/opencv_contrib.git
echo "Downloading opencv... (10/10)"
git clone https://github.com/opencv/opencv.git


# Install directory for all dependencies
mkdir -p $prefix/arm64-v8a

#CMake 3.17 for VTK
echo "Install cmake3.17..."
tar -xzf cmake-3.17.0-Linux-x86_64.tar.gz
rm cmake-3.17.0-Linux-x86_64.tar.gz

# Boost
echo "Install boost..."
tar -xzf boost_1_59_0.tar.gz
cd boost_1_59_0
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/1733253195bc4d4d9b7f9eda1e60628dc1e51429/BoostConfig.cmake.in
wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/1733253195bc4d4d9b7f9eda1e60628dc1e51429/CMakeLists.txt
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" ..
make $cpus
make install
cd $pwd
rm -r boost_1_59_0.tar.gz boost_1_59_0

# eigen
echo "Install eigen..."
tar -xzf 3.3.9.tar.gz
cd eigen-3.3.9
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" ..
make $cpus
make install
cd $pwd
rm -r 3.3.9.tar.gz eigen-3.3.9

# FLANN
echo "Install flann..."
cd flann
wget -nv https://gist.githubusercontent.com/matlabbe/cacff9f8271d0c42acd622939a26cab4/raw/85baf4927b32844ebd7f8eccce421bde181cd190/flann_1_8_4_android_fix.patch
git apply flann_1_8_4_android_fix.patch
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" ..
make $cpus
make install
cd $pwd
rm -rf flann

# GTSAM
echo "Install gtsam..."
cd gtsam
git checkout fbb9d3bdda8b88df51896bc401bfd170573e66f5
# patch
wget -nv https://gist.github.com/matlabbe/726b490c658afd3293f4b3f2f501b863/raw/df09fc8e238a495d66b062d92dc1c1fb20a581e8/gtsam_GKlib_android_fix.patch
git apply gtsam_GKlib_android_fix.patch
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make $cpus
make install
cd $pwd
rm -rf gtsam

# g2o
echo "Install g2o..."
cd g2o
git checkout a3f7706bdbb849b2808dc3e1b7aee189f63b498e
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF ..
make $cpus
make install
cd $pwd
rm -rf g2o

# VTK
echo "Install VTK..."
cd VTK
git checkout tags/v8.2.0
wget https://gist.github.com/matlabbe/e217259fb8ece9ee6daf5a8f70e896a0/raw/2214b503a537d6431d764526b5b780f07d6f168d/vtk_8_2_0_android_r21_fix.patch
git apply vtk_8_2_0_android_r21_fix.patch
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_ANDROID_BUILD=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DANDROID_ARCH_ABI=arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" ..
make $cpus
cp -r CMakeExternals/Install/vtk-android/* $prefix/arm64-v8a/.
cd $pwd
rm -rf VTK

# PCL
echo "Install pcl..."
cd pcl
git checkout tags/pcl-1.8.0
# patch
wget https://gist.github.com/matlabbe/41812e50e459b2f27b331a2343569e5d/raw/b2fc0c4d1cfffb3a9f2811abae782e317c539bfb/pcl_1_8_0_vtk_android_support.patch
git apply pcl_1_8_0_vtk_android_support.patch
mkdir build
cd build
# do it 2 times because there is a cmake error on the first time and not the second time!?
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF ..
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF ..
make $cpus
make install
cd $pwd
rm -rf pcl

# make sure opencv is using the shared version of zlib
# see https://github.com/android/ndk/issues/1179
mv $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libz.a $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libz.a.back

# OpenCV
echo "Install OpenCV..."
cd opencv_contrib
git checkout tags/4.5.5
cd $pwd
cd opencv
git checkout tags/4.5.5
mkdir build
cd build
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_NATIVE_API_LEVEL -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DOPENCV_EXTRA_MODULES_PATH=$pwd/opencv_contrib/modules -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DBUILD_opencv_structured_light=OFF -DBUILD_ANDROID_PROJECTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DBUILD_ANDROID_EXAMPLES=OFF -DWITH_PROTOBUF=OFF -DBUILD_opencv_stereo=OFF -DBUILD_JAVA=OFF -DWITH_QUIRC=OFF -DBUILD_opencv_js_bindings_generator=OFF -DBUILD_opencv_objc_bindings_generator=OFF -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_xobjdetect=OFF ..
make $cpus
make install
cd $pwd
rm -rf opencv opencv_contrib

echo "Strip libraries..."
$ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android-strip -g -S -d --strip-debug --verbose /opt/android/arm64-v8a/lib/*.a
$ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android-strip -g -S -d --strip-debug --verbose /opt/android/arm64-v8a/sdk/native/staticlibs/arm64-v8a/*.a

#!/bin/bash

set -euxo pipefail

# Tested on Apple Silicon Mac, with cmake 3.19.2.

mkdir Libraries
cd Libraries
pwd=$(pwd)
prefix=$pwd
sysroot=iphoneos
#sysroot=iphonesimulator


# Install directory for all dependencies
mkdir -p $prefix

# openmp
# based on https://github.com/Homebrew/homebrew-core/blob/HEAD/Formula/libomp.rb
#curl -L https://github.com/llvm/llvm-project/releases/download/llvmorg-11.1.0/openmp-11.1.0.src.tar.xz -o openmp-11.1.0.src.tar.xz
#tar -xzf openmp-11.1.0.src.tar.xz
#cd openmp-11.1.0.src
#curl -L https://raw.githubusercontent.com/Homebrew/formula-patches/7e2ee1d7/libomp/arm.patch -o arm.patch
#patch -p1 < arm.patch
#mkdir build
#cd build
#cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DLIBOMP_INSTALL_ALIASES=OFF -DLIBOMP_ENABLE_SHARED=OFF ..
#cmake --build . --config Release -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
#cp runtime/src/Release-iphoneos/libomp.a runtime/src/.
#cmake --build . --config Release --target install -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"

# Boost
echo "wget boost..."
curl -L https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz -o boost_1_59_0.tar.gz
tar -xzf boost_1_59_0.tar.gz
cd boost_1_59_0
curl -L https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/489ff2869eccd6f8d03ffb9090ef839108762741/BoostConfig.cmake.in -o BoostConfig.cmake.in
curl -L https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/b07fe7d4e5dfe5f1d110c733e5cf660d79a26378/CMakeLists.txt -o CMakeLists.txt
mkdir build
cd build
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix ..
cmake --build . --config Release
cmake --build . --config Release --target install
cd $pwd
#rm -r boost_1_59_0.tar.gz boost_1_59_0

# eigen
echo "wget eigen..."
curl -L https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o 3.3.9.tar.gz
tar -xzf 3.3.9.tar.gz
cd eigen-3.3.9
mkdir build
cd build
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix ..
cmake --build . --config Release
cmake --build . --config Release --target install
cd $pwd
#rm -r 3.3.9.tar.gz eigen-3.3.9

# FLANN
echo "wget flann..."
git clone https://github.com/flann-lib/flann.git
cd flann
git checkout 1.8.4
curl -L https://gist.githubusercontent.com/matlabbe/c858ba36fb85d5e44d8667dfb3543e12/raw/8fc40aa9bc3267604869444020476a49f14ab424/flann_ios.patch  -o flann_ios.patch
git apply flann_ios.patch
mkdir build
cd build
# comment "add_subdirectory( test )" in top CMakeLists.txt
# comment "add_subdirectory( doc )" in top CMakeLists.txt
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_C_BINDINGS=OFF  ..
cmake --build . --config Release -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cmake --build . --config Release --target install -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cd $pwd
#rm -r flann

# GTSAM
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout fbb9d3bdda8b88df51896bc401bfd170573e66f5
# patch
curl -L https://gist.github.com/matlabbe/76d658dddb841b3355ae3a6e32850cd8/raw/7033cba1c89097b0c830651d7277c04dc92cbdd9/gtsam_GKlib_ios_fix.patch -o gtsam_GKlib_ios_fix.patch
git apply gtsam_GKlib_ios_fix.patch
mkdir build
cd build
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WRAP_SERIALIZATION=OFF -DGTSAM_BUILD_WRAP=OFF -DGTSAM_INSTALL_CPPUNITLITE=OFF -DCMAKE_FIND_ROOT_PATH=$prefix ..
cmake --build . --config Release -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cmake --build . --config Release --target install -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cd $pwd
#rm -rf gtsam

# g2o
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout a3f7706bdbb849b2808dc3e1b7aee189f63b498e
# patch
curl -L https://gist.github.com/matlabbe/b9ccfeae8f0744b275cab23510872680/raw/a58e06accba3976420d4b61341685c123193810e/g2o_ios_fix.patch -o g2o_ios_fix.patch
git apply g2o_ios_fix.patch
mkdir build
cd build
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DCMAKE_FIND_ROOT_PATH=$prefix ..
cmake --build . --config Release -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cmake --build . --config Release --target install -- CODE_SIGN_IDENTITY="" CODE_SIGNING_REQUIRED="NO" CODE_SIGN_ENTITLEMENTS=""  CODE_SIGNING_ALLOWED="NO"
cd $pwd
#rm -rf g2o

# VTK
git clone https://github.com/Kitware/VTK.git
cd VTK
git checkout tags/v8.2.0
git cherry-pick bf3ae8072df2393c7270509bae41be0776826346
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_FRAMEWORK_INSTALL_PREFIX=$prefix/lib -DIOS_DEVICE_ARCHITECTURES="arm64" -DIOS_SIMULATOR_ARCHITECTURES="" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_IOS_BUILD=ON -DModule_vtkFiltersModeling=ON ..
# For iphonesimulator: add -DIOS_DEVICE_ARCHITECTURES=""
cmake --build . --config Release
cd $pwd
#rm -rf VTK

# PCL
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout tags/pcl-1.11.1
# patch
curl -L https://gist.github.com/matlabbe/f3ba9366eb91e1b855dadd2ddce5746d/raw/6869cf26211ab15492599e557b0e729b23b2c119/pcl_1_11_1_vtk_ios_support.patch -o pcl_1_11_1_vtk_ios_support.patch
git apply pcl_1_11_1_vtk_ios_support.patch
mkdir build
cd build
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF -DCMAKE_FIND_ROOT_PATH=$prefix ..
cmake --build . --config Release
cmake --build . --config Release --target install
cd $pwd
#rm -rf pcl

# OpenCV
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout tags/3.4.2
cd $pwd
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout tags/3.4.2
curl -L https://gist.githubusercontent.com/matlabbe/fdc3ab4854f3a68fbde7277f543b4e5b/raw/f340839c09165056d3845645df24b76507542fd2/opencv_ios.patch -o opencv_ios.patch
git apply opencv_ios.patch
mkdir build
cd build
# add "add_definitions(-DPNG_ARM_NEON_OPT=0)" in 3rdparty/libpng/CMakeLists.txt
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DOPENCV_EXTRA_MODULES_PATH=$prefix/opencv_contrib/modules -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_xobjdetect=OFF -DBUILD_opencv_stereo=OFF ..
cmake --build . --config Release
cmake --build . --config Release --target install
cd $pwd
#rm -rf opencv opencv_contrib

mkdir rtabmap
cd rtabmap
cmake -DANDROID_PREBUILD=ON ../../../../..
cmake --build . --config Release
mkdir ios
cd ios
cmake -G Xcode -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_SYSROOT=$sysroot -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 -DCMAKE_C_FLAGS=-fembed-bitcode -DCMAKE_CXX_FLAGS=-fembed-bitcode -DCMAKE_INSTALL_PREFIX=$prefix -DCMAKE_FIND_ROOT_PATH=$prefix -DWITH_QT=OFF -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DWITH_TORO=OFF -DWITH_VERTIGO=OFF -DWITH_MADGWICK=OFF -DWITH_ORB_OCTREE=OFF  -DBUILD_EXAMPLES=OFF ../../../../../..
cmake --build . --config Release
cmake --build . --config Release --target install

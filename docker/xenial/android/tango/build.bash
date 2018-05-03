#!/bin/bash

if [ $# -ne 1 ]; then
    echo "build.bash android_install_prefix   # Example: build.bash /opt/android"
    exit 1
fi

prefix=$1

# get rtabmap
git clone https://github.com/introlab/rtabmap.git rtabmap-tango

# tango
wget 'https://docs.google.com/uc?authuser=0&id=12rHHkYM5k-UnQn-xGXs9JqYWhSXrgJr3&export=download' -O TangoSDK_Ikariotikos_C.zip
unzip -qq TangoSDK_Ikariotikos_C.zip
rm TangoSDK_Ikariotikos_C.zip
cp -r lib_tango_client_api/include/* $prefix/armeabi-v7a/include/.
cp -r lib_tango_client_api/include/* $prefix/arm64-v8a/include/.
cp -r lib_tango_client_api/lib/armeabi-v7a/* $prefix/armeabi-v7a/lib/.
cp -r lib_tango_client_api/lib/arm64-v8a/* $prefix/arm64-v8a/lib/.
rm -r lib_tango_client_api
wget 'https://docs.google.com/uc?authuser=0&id=1AqVuEVu5284X6OgrGWu12VTrx4pY99Jb&export=download' -O TangoSupport_Ikariotikos_C.zip
unzip -qq TangoSupport_Ikariotikos_C.zip
rm TangoSupport_Ikariotikos_C.zip
cp -r lib_tango_support_api/include/* $prefix/armeabi-v7a/include/.
cp -r lib_tango_support_api/include/* $prefix/arm64-v8a/include/.
cp -r lib_tango_support_api/lib/armeabi-v7a/* $prefix/armeabi-v7a/lib/.
cp -r lib_tango_support_api/lib/arm64-v8a/* $prefix/arm64-v8a/lib/.
cp -r lib_tango_support_api/lib/* rtabmap-tango/app/android/jni/third-party/lib/.
rm -r lib_tango_support_api
wget 'https://docs.google.com/uc?authuser=0&id=1s5iPJ7xiridj9Jj--gCy2XiQFniheVm6&export=download' -O TangoSDK_Ikariotikos_Java.jar
mv TangoSDK_Ikariotikos_Java.jar rtabmap-tango/app/android/libs/.

# resource tool
cd rtabmap-tango/build
cmake -DANDROID_PREBUILD=ON ..
make
cd ../..

# rtabmap
mkdir rtabmap-tango/build/armeabi-v7a
cd rtabmap-tango/build/armeabi-v7a
cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake_modules/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$prefix/armeabi-v7a/sdk/native/jni -DCMAKE_INSTALL_PREFIX=$prefix/armeabi-v7a ../..
make

cd ../../..
mkdir rtabmap-tango/build/arm64-v8a
cd rtabmap-tango/build/arm64-v8a
cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake_modules/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$prefix/arm64-v8a/sdk/native/jni -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a ../..
make

# package with binaries of both architectures
cp -r ../armeabi-v7a/app/android/libs/armeabi-v7a app/android/libs/.
make

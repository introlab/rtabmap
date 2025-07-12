#!/bin/bash

set -e

if [ $# -ne 2 ]; then
    echo "rtabmap.bash android_install_prefix api_level (23 for tango, 24 for arengine)   # Example: build.bash /opt/android 24"
    exit 1
fi

prefix=$1
api=$2

# copy required jars
cp /opt/android/lib/*.jar app/android/libs/.

# resource tool
cd build
cmake -DANDROID_PREBUILD=ON ..
make

# rtabmap
mkdir arm64-v8a
cd arm64-v8a
cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$api -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DOpenCV_DIR=$prefix/arm64-v8a/sdk/native/jni ../..
make
make clean




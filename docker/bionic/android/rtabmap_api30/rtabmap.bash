#!/bin/bash

if [ $# -ne 2 ]; then
    echo "rtabmap.bash android_install_prefix api_level (23 for tango, 24 for arengine)   # Example: build.bash /opt/android 24"
    exit 1
fi

prefix=$1
api=$2
pwd=$(pwd)


# get rtabmap
git clone https://github.com/introlab/rtabmap.git rtabmap-tango
cd 

# tango
wget 'https://docs.google.com/uc?authuser=0&id=12rHHkYM5k-UnQn-xGXs9JqYWhSXrgJr3&export=download' -O TangoSDK_Ikariotikos_C.zip
unzip -qq TangoSDK_Ikariotikos_C.zip
rm TangoSDK_Ikariotikos_C.zip
cp -r lib_tango_client_api/include/* $prefix/arm64-v8a/include/.
cp -r lib_tango_client_api/lib/arm64-v8a/* $prefix/arm64-v8a/lib/.
rm -r lib_tango_client_api
wget 'https://docs.google.com/uc?authuser=0&id=1AqVuEVu5284X6OgrGWu12VTrx4pY99Jb&export=download' -O TangoSupport_Ikariotikos_C.zip
unzip -qq TangoSupport_Ikariotikos_C.zip
rm TangoSupport_Ikariotikos_C.zip
cp -r lib_tango_support_api/include/* $prefix/arm64-v8a/include/.
cp -r lib_tango_support_api/lib/arm64-v8a/* $prefix/arm64-v8a/lib/.
rm -r lib_tango_support_api
wget 'https://docs.google.com/uc?authuser=0&id=1s5iPJ7xiridj9Jj--gCy2XiQFniheVm6&export=download' -O TangoSDK_Ikariotikos_Java.jar
mv TangoSDK_Ikariotikos_Java.jar rtabmap-tango/app/android/libs/.

# ARCore
wget 'https://docs.google.com/uc?authuser=0&id=1VsibeqRYpS5pjmrG-vYTXyiPg8kbIfVN&export=download' -O arcore.zip
unzip -qq arcore.zip
rm arcore.zip
cp -r arcore1_18/include/* $prefix/arm64-v8a/include/.
cp -r arcore1_18/arm64-v8a/* $prefix/arm64-v8a/lib/.
cp arcore1_18/*.jar rtabmap-tango/app/android/libs/.
rm -r arcore1_18

# AREngine
wget 'https://docs.google.com/uc?authuser=0&id=1rdaD2Z1QBv-SUeUy0oBmg3C2odfxTHgR&export=download' -O arengine.zip
unzip -qq arengine.zip
rm arengine.zip
cp -r arengine/include/* $prefix/arm64-v8a/include/.
cp -r arengine/arm64-v8a/* $prefix/arm64-v8a/lib/.
cp arengine/*.jar rtabmap-tango/app/android/libs/.
rm -r arengine

# resource tool
cd rtabmap-tango/build
cmake -DANDROID_PREBUILD=ON ..
make
cd ../..

# rtabmap
mkdir rtabmap-tango/build/arm64-v8a
cd rtabmap-tango/build/arm64-v8a
$pwd/cmake-3.17.0-Linux-x86_64/bin/cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$api -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$prefix/arm64-v8a -DCMAKE_FIND_ROOT_PATH="$prefix/arm64-v8a/bin;$prefix/arm64-v8a;$prefix/arm64-v8a/share" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DOpenCV_DIR=$prefix/arm64-v8a/sdk/native/jni ../..
make
make clean




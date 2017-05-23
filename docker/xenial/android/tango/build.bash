#!/bin/bash

# get rtabmap
git clone https://github.com/introlab/rtabmap.git rtabmap-tango

# tango
wget https://developers.google.com/tango/downloads/TangoSDK_Hopak_C.zip
unzip -qq TangoSDK_Hopak_C.zip
rm TangoSDK_Hopak_C.zip
cp -r lib_tango_client_api/include/* /opt/android/armeabi-v7a/include/.
cp -r lib_tango_client_api/include/* /opt/android/arm64-v8a/include/.
cp -r lib_tango_client_api/lib/armeabi-v7a/* /opt/android/armeabi-v7a/lib/.
cp -r lib_tango_client_api/lib/arm64-v8a/* /opt/android/arm64-v8a/lib/.
rm -r lib_tango_client_api
wget https://developers.google.com/tango/downloads/TangoSupport_Hopak_C.zip
unzip -qq TangoSupport_Hopak_C.zip
rm TangoSupport_Hopak_C.zip
cp -r lib_tango_support_api/include/* /opt/android/armeabi-v7a/include/.
cp -r lib_tango_support_api/include/* /opt/android/arm64-v8a/include/.
cp -r lib_tango_support_api/lib/armeabi-v7a/* /opt/android/armeabi-v7a/lib/.
cp -r lib_tango_support_api/lib/arm64-v8a/* /opt/android/arm64-v8a/lib/.
cp -r lib_tango_support_api/lib/* rtabmap-tango/app/android/jni/third-party/lib/.
rm -r lib_tango_support_api
wget https://developers.google.com/tango/downloads/TangoSDK_Hopak_Java.jar
mv TangoSDK_Hopak_Java.jar rtabmap-tango/app/android/libs/.

# rtabmap
mkdir rtabmap-tango/build/armeabi-v7a
cd rtabmap-tango/build/armeabi-v7a
cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake_modules/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/android/armeabi-v7a/sdk/native/jni -DCMAKE_INSTALL_PREFIX=/opt/android/armeabi-v7a ../..
make

cd
mkdir rtabmap-tango/build/arm64-v8a
cd rtabmap-tango/build/arm64-v8a
cmake -DCMAKE_TOOLCHAIN_FILE=../../cmake_modules/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/android/arm64-v8a/sdk/native/jni -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a ../..
make

# package with binaries of both architectures
cp -r ../armeabi-v7a/app/android/libs/armeabi-v7a app/android/libs/armeabi-v7a
make
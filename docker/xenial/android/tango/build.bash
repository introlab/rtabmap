#!/bin/bash

apt-get install -y lib32stdc++6 lib32z1

# get rtabmap
git clone https://github.com/introlab/rtabmap.git rtabmap-tango

# tango
wget https://developers.google.com/tango/downloads/TangoSDK_Farandole_C.zip
unzip -qq TangoSDK_Farandole_C.zip
cp -r lib_tango_client_api/* /opt/android/.
rm -r lib_tango_client_api
wget https://developers.google.com/tango/downloads/TangoSupport_Farandole_C.zip
unzip -qq TangoSupport_Farandole_C.zip
cp -r lib_tango_support_api/* /opt/android/.
cp -r lib_tango_support_api/lib/* rtabmap-tango/app/android/jni/third-party/lib/.
rm -r lib_tango_support_api
wget https://developers.google.com/tango/downloads/TangoSDK_Farandole_Java.jar
mv TangoSDK_Farandole_Java.jar rtabmap-tango/app/android/libs/.

# rtabmap
cd rtabmap-tango/build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake_modules/android.toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/opt/OpenCV-android-sdk/sdk/native/jni -DCMAKE_INSTALL_PREFIX=/opt/android ..
make

#!/bin/bash
set -e

echo "Running post-start initialization..."

# copy required jars
cp /opt/android/lib/*.jar app/android/libs/.

mkdir -p build_android/arm64-v8a

# resource tool
cd build_android
cmake -DANDROID_PREBUILD=ON ..
make

echo -e "\nTo build the APK, do (adjust API number):"
echo -e '\nexport ANDROID_API=30 && cd build_android/arm64-v8a && cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=$ANDROID_API -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DOpenCV_DIR=/opt/android/arm64-v8a/sdk/native/jni ../..\nmake -j6\n'
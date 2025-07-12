# Image: introlab3it/rtabmap:android-noble-deps

FROM ubuntu:24.04

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y \
      git unzip wget ant cmake \
      g++ lib32stdc++6 lib32z1 \
      software-properties-common \
      freeglut3-dev \
      openjdk-8-jdk openjdk-8-jre \
      curl

ENV ANDROID_HOME=/opt/android-sdk
ENV PATH=$PATH:/opt/android-sdk/cmdline-tools/latest/bin:/opt/android-sdk/tools:/opt/android-sdk/platform-tools:/opt/android-sdk/ndk/21.4.7075529
ENV ANDROID_NDK=/opt/android-sdk/ndk/21.4.7075529
ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64

WORKDIR /root/

# Setup android sdk
RUN wget -nv https://dl.google.com/android/repository/commandlinetools-linux-7583922_latest.zip && \
    unzip -qq commandlinetools-linux-7583922_latest.zip && \
    rm commandlinetools-linux-7583922_latest.zip && \
    mkdir $ANDROID_HOME && \
    mkdir $ANDROID_HOME/cmdline-tools && \
    mv cmdline-tools $ANDROID_HOME/cmdline-tools/latest
# We should use build-tools <=30 to avoid dx missing error
RUN echo y | sdkmanager --install "platform-tools" "platforms;android-23" "platforms;android-24" "platforms;android-26" "platforms;android-30" "build-tools;30.0.3" "ndk;21.4.7075529"

# we need <=r25 tools to use "android" command (now deprecated)
RUN wget -nv http://dl-ssl.google.com/android/repository/tools_r25.2.5-linux.zip && \
    unzip -qq tools_r25.2.5-linux.zip && \
    mv tools $ANDROID_HOME/. && \
    rm tools_r25.2.5-linux.zip

##############
# Dependencies (took from docker/bionic/android/deps.bash)
##############

# Install directory for all dependencies
RUN mkdir -p /opt/android/arm64-v8a

# Boost
RUN echo "Install boost..." && \
    wget -nv https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz && \
    tar -xzf boost_1_59_0.tar.gz && \
    cd boost_1_59_0 && \
    wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/1733253195bc4d4d9b7f9eda1e60628dc1e51429/BoostConfig.cmake.in && \
    wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/1733253195bc4d4d9b7f9eda1e60628dc1e51429/CMakeLists.txt && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" ..  && \
    make -j4 && \
    make install && \
    cd /root  && \
    rm -r boost_1_59_0.tar.gz boost_1_59_0

# eigen
RUN echo "Install eigen..." && \
    curl -L https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o 3.3.9.tar.gz && \
    tar -xzf 3.3.9.tar.gz && \
    cd eigen-3.3.9 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" ..  && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -r 3.3.9.tar.gz eigen-3.3.9

# FLANN
RUN echo "Install flann..." && \
    git clone -b 1.8.4 https://github.com/mariusmuja/flann.git && \
    cd flann && \
    wget -nv https://gist.githubusercontent.com/matlabbe/cacff9f8271d0c42acd622939a26cab4/raw/85baf4927b32844ebd7f8eccce421bde181cd190/flann_1_8_4_android_fix.patch && \
    git apply flann_1_8_4_android_fix.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" .. && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -rf flann

# GTSAM
RUN echo "Install gtsam..." && \
    git clone https://bitbucket.org/gtborg/gtsam.git && \
    cd gtsam && \
    git checkout fbb9d3bdda8b88df51896bc401bfd170573e66f5 && \
    wget -nv https://gist.github.com/matlabbe/726b490c658afd3293f4b3f2f501b863/raw/df09fc8e238a495d66b062d92dc1c1fb20a581e8/gtsam_GKlib_android_fix.patch && \
    git apply gtsam_GKlib_android_fix.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DMETIS_SHARED=OFF -DGTSAM_BUILD_STATIC_LIBRARY=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -rf gtsam

# g2o
RUN echo "Install g2o..." && \
    git clone https://github.com/RainerKuemmerle/g2o.git && \
    cd g2o && \
    git checkout a3f7706bdbb849b2808dc3e1b7aee189f63b498e && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DBUILD_LGPL_SHARED_LIBS=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF .. && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -rf g2o

# VTK
RUN echo "Install VTK..." && \
    git clone https://github.com/Kitware/VTK.git && \
    cd VTK && \
    git checkout tags/v8.2.0 && \
    wget https://gist.github.com/matlabbe/e217259fb8ece9ee6daf5a8f70e896a0/raw/2214b503a537d6431d764526b5b780f07d6f168d/vtk_8_2_0_android_r21_fix.patch && \
    git apply vtk_8_2_0_android_r21_fix.patch && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_ANDROID_BUILD=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DANDROID_ARCH_ABI=arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" .. && \
    make -j4 && \
    cp -r CMakeExternals/Install/vtk-android/* /opt/android/arm64-v8a/. && \
    cd /root && \
    rm -rf VTK

# PCL
RUN echo "Install pcl..." && \
    git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    git checkout tags/pcl-1.8.0 && \
    wget https://gist.github.com/matlabbe/41812e50e459b2f27b331a2343569e5d/raw/b2fc0c4d1cfffb3a9f2811abae782e317c539bfb/pcl_1_8_0_vtk_android_support.patch && \
    git apply pcl_1_8_0_vtk_android_support.patch && \
    mkdir build && \
    cd build && \
    # do it 2 times because there is a cmake error on the first time and not the second time!?
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF .. || true && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_VTK=ON -DPCL_SHARED_LIBS=OFF .. && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -rf pcl

# make sure opencv is using the shared version of zlib
# see https://github.com/android/ndk/issues/1179
RUN mv $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libz.a $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libz.a.back

# OpenCV
RUN echo "Install OpenCV..." && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv_contrib && \
    git checkout tags/4.5.5 && \
    cd /root && \
    cd opencv && \
    git checkout tags/4.5.5 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_NDK=$ANDROID_NDK -DANDROID_NATIVE_API_LEVEL=23 -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/android/arm64-v8a -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DBUILD_opencv_structured_light=OFF -DBUILD_ANDROID_PROJECTS=OFF -DOPENCV_ENABLE_NONFREE=ON -DBUILD_ANDROID_EXAMPLES=OFF -DWITH_PROTOBUF=OFF -DBUILD_opencv_stereo=OFF -DBUILD_JAVA=OFF -DWITH_QUIRC=OFF -DBUILD_opencv_js_bindings_generator=OFF -DBUILD_opencv_objc_bindings_generator=OFF -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_xobjdetect=OFF .. && \
    make -j4 && \
    make install && \
    cd /root && \
    rm -rf opencv opencv_contrib

RUN echo "Strip libraries..." && \
    $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android-strip -g -S -d --strip-debug --verbose /opt/android/arm64-v8a/lib/*.a && \
    $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android-strip -g -S -d --strip-debug --verbose /opt/android/arm64-v8a/sdk/native/staticlibs/arm64-v8a/*.a

RUN mkdir /opt/android/lib

# tango
RUN wget 'https://docs.google.com/uc?authuser=0&id=12rHHkYM5k-UnQn-xGXs9JqYWhSXrgJr3&export=download' -O TangoSDK_Ikariotikos_C.zip && \
    unzip -qq TangoSDK_Ikariotikos_C.zip && \
    rm TangoSDK_Ikariotikos_C.zip && \
    cp -r lib_tango_client_api/include/* /opt/android/arm64-v8a/include/. && \
    cp -r lib_tango_client_api/lib/arm64-v8a/* /opt/android/arm64-v8a/lib/. && \
    rm -r lib_tango_client_api && \
    wget 'https://docs.google.com/uc?authuser=0&id=1AqVuEVu5284X6OgrGWu12VTrx4pY99Jb&export=download' -O TangoSupport_Ikariotikos_C.zip && \
    unzip -qq TangoSupport_Ikariotikos_C.zip && \
    rm TangoSupport_Ikariotikos_C.zip && \
    cp -r lib_tango_support_api/include/* /opt/android/arm64-v8a/include/. && \
    cp -r lib_tango_support_api/lib/arm64-v8a/* /opt/android/arm64-v8a/lib/. && \
    rm -r lib_tango_support_api && \
    wget 'https://docs.google.com/uc?authuser=0&id=1s5iPJ7xiridj9Jj--gCy2XiQFniheVm6&export=download' -O TangoSDK_Ikariotikos_Java.jar && \
    mv TangoSDK_Ikariotikos_Java.jar /opt/android/lib/

# ARCore
RUN wget 'https://docs.google.com/uc?authuser=0&id=1VsibeqRYpS5pjmrG-vYTXyiPg8kbIfVN&export=download' -O arcore.zip && \
    unzip -qq arcore.zip && \
    rm arcore.zip && \
    cp -r arcore1_18/include/* /opt/android/arm64-v8a/include/. && \
    cp -r arcore1_18/arm64-v8a/* /opt/android/arm64-v8a/lib/. && \
    cp arcore1_18/*.jar /opt/android/lib/ && \
    rm -r arcore1_18

# AREngine
RUN wget 'https://docs.google.com/uc?authuser=0&id=1rdaD2Z1QBv-SUeUy0oBmg3C2odfxTHgR&export=download' -O arengine.zip && \
    unzip -qq arengine.zip && \
    rm arengine.zip && \
    cp -r arengine/include/* /opt/android/arm64-v8a/include/. && \
    cp -r arengine/arm64-v8a/* /opt/android/arm64-v8a/lib/. && \
    cp arengine/*.jar /opt/android/lib/ && \
    rm -r arengine
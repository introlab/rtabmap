# Image: introlab3it/rtabmap:android-deps

FROM ubuntu:16.04

# Install build dependencies
RUN apt-get update && apt-get install -y \
      git \
      cmake \
      g++ \
      software-properties-common \
      freeglut3-dev

ENV ANDROID_HOME=/opt/android-sdk
ENV ANDROID_NATIVE_API_LEVEL=android-19
ENV ANDROID_NDK=/opt/android-ndk-r14
ENV PATH=$PATH:/opt/android-sdk/tools:/opt/android-sdk/platform-tools:/opt/android-ndk-r14

WORKDIR /root/

ADD setup_android_env.bash /root/setup_android_env.bash
RUN chmod +x setup_android_env.bash
RUN /bin/bash -c "./setup_android_env.bash"

ADD install_deps.bash /root/install_deps.bash
RUN chmod +x install_deps.bash
RUN /bin/bash -c "./install_deps.bash /opt/android"

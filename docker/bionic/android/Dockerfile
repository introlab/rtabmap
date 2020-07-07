# Image: introlab3it/rtabmap:android-deps

FROM ubuntu:18.04

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y \
      git unzip wget ant cmake \
      g++ lib32stdc++6 lib32z1 \
      software-properties-common \
      freeglut3-dev \
      openjdk-8-jdk openjdk-8-jre

ENV ANDROID_NDK_VERSION=r21
ENV ANDROID_HOME=/opt/android-sdk
ENV PATH=$PATH:/opt/android-sdk/tools:/opt/android-sdk/platform-tools:/opt/android-ndk-$ANDROID_NDK_VERSION
ENV ANDROID_NDK=/opt/android-ndk-$ANDROID_NDK_VERSION
ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64

WORKDIR /root/

# Setup android sdk
RUN wget -nv https://dl.google.com/android/repository/tools_r25.2.3-linux.zip && \
    unzip -qq tools_r25.2.3-linux.zip && \
    rm tools_r25.2.3-linux.zip && \
    mkdir $ANDROID_HOME && \
    mv tools $ANDROID_HOME/.
RUN echo y | android update sdk --no-ui --all --filter platform-tools,android-23,android-24,android-26,build-tools-29.0.3

# Setup android ndk
RUN wget -nv https://dl.google.com/android/repository/android-ndk-$ANDROID_NDK_VERSION-linux-x86_64.zip && \
    unzip -qq android-ndk-$ANDROID_NDK_VERSION-linux-x86_64.zip && \
    rm android-ndk-$ANDROID_NDK_VERSION-linux-x86_64.zip && \
    mv android-ndk-$ANDROID_NDK_VERSION /opt/.

ADD deps.bash /root/deps.bash
RUN chmod +x deps.bash
RUN /bin/bash -c "./deps.bash /opt/android"

# Image: introlab3it/rtabmap:android-deps

FROM ubuntu:18.04

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
RUN echo y | sdkmanager --install "platform-tools" "platforms;android-23" "platforms;android-24" "platforms;android-26" "platforms;android-30" "build-tools;30.0.3" "ndk;21.4.7075529" && \
    rm -r $ANDROID_HOME/tools

# we need <=r25 tools to use "android" command (now deprecated)
RUN wget -nv http://dl-ssl.google.com/android/repository/tools_r25.2.5-linux.zip && \
    unzip -qq tools_r25.2.5-linux.zip && \
    mv tools $ANDROID_HOME/. && \
    rm tools_r25.2.5-linux.zip

ADD deps.bash /root/deps.bash
RUN chmod +x deps.bash
RUN /bin/bash -c "./deps.bash /opt/android"

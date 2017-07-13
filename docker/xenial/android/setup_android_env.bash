#!/bin/bash

# Setup java 1.8
apt-get update
apt-get install -y --no-install-recommends apt-utils
apt-get install -y unzip wget ant
apt-get install -y default-jre default-jdk
apt-get install -y lib32stdc++6 lib32z1

# Setup android sdk
echo "wget android-sdk..."
wget -nv https://dl.google.com/android/repository/tools_r25.2.3-linux.zip
unzip -qq tools_r25.2.3-linux.zip
rm tools_r25.2.3-linux.zip
mkdir $ANDROID_HOME
mv tools $ANDROID_HOME/.
#android list sdk --all --extended
echo y | android update sdk --no-ui --all --filter platform-tools,android-19,build-tools-19.1.0,android-21,build-tools-21.1.0

# Setup android ndk
echo "wget android-ndk..."
wget -nv https://dl.google.com/android/repository/android-ndk-r14-linux-x86_64.zip
unzip -qq android-ndk-r14-linux-x86_64.zip
rm android-ndk-r14-linux-x86_64.zip
mv android-ndk-r14 $ANDROID_NDK

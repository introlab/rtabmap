# Image: introlab3it/rtabmap:androidXX

FROM introlab3it/rtabmap:android-noble-deps

ARG API_VERSION=23

# Copy current source code
COPY . /root/rtabmap-tango

WORKDIR /root/rtabmap-tango

RUN /bin/bash -c "./docker/noble/android/rtabmap_apiXX/rtabmap.bash /opt/android $API_VERSION"


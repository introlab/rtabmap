# Image: introlab3it/rtabmap:androidXX

FROM introlab3it/rtabmap:android-deps

ARG API_VERSION=23

WORKDIR /root/

# Copy current source code
COPY . /root/rtabmap-tango

RUN /bin/bash -c "./rtabmap-tango/docker/bionic/android/rtabmap_apiXX/rtabmap.bash /opt/android $API_VERSION"


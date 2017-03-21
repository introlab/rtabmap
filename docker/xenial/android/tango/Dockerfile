FROM introlab3it/rtabmap-android-deps

WORKDIR /root/

ADD build.bash /root/build.bash
RUN chmod +x build.bash
RUN /bin/bash -c "./build.bash"

WORKDIR /root/rtabmap-android/build/app/android/bin

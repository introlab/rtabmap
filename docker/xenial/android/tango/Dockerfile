# Image: introlab3it/rtabmap:tango-api19

FROM introlab3it/rtabmap:android-deps-api19

WORKDIR /root/

ADD build.bash /root/build.bash
RUN chmod +x build.bash
RUN /bin/bash -c "./build.bash /opt/android"

WORKDIR /root/

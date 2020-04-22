# Image: introlab3it/rtabmap:android24

FROM introlab3it/rtabmap:android-deps

WORKDIR /root/

ADD rtabmap.bash /root/rtabmap.bash
RUN chmod +x rtabmap.bash
RUN /bin/bash -c "./rtabmap.bash /opt/android 24"


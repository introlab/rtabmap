# Image: introlab3it/rtabmap:android30

FROM introlab3it/rtabmap:android-deps

WORKDIR /root/

ARG CACHE_DATE=2016-01-01
ADD rtabmap.bash /root/rtabmap.bash
RUN chmod +x rtabmap.bash
RUN /bin/bash -c "./rtabmap.bash /opt/android 30"


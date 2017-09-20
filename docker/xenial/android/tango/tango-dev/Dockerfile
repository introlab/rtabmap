# Image: introlab3it/rtabmap:tango-dev

FROM introlab3it/rtabmap:tango

WORKDIR /root/

RUN apt-get update && apt-get install -y \
      vim \
      eclipse \
      eclipse-cdt \
      eclipse-cdt-jni
      
RUN cp ~/rtabmap-tango/build/arm64-v8a/app/android/project.properties ~/rtabmap-tango/app/android/.

# Manual steps:
# - Install ADT plugin from Eclipse: https://stuff.mit.edu/afs/sipb/project/android/docs/sdk/installing/installing-adt.html
# - Import rtabmap project, set build command: "make -C ${ProjDirPath}/build/arm64-v8a VERBOSE=true"
# - Create Android project from source code and select rtabmap-tango/app/android directory.
# Image: introlab3it/rtabmap:jammy

FROM introlab3it/rtabmap:jammy-deps

# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map && chmod 777 /root/Documents/RTAB-Map

# Copy current source code
COPY . /root/rtabmap

ARG RUN_TESTS=0

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DWITH_OPENGV=ON -DBUILD_TESTING=$RUN_TESTS .. && \
    make -j4 && \
    if [ "$RUN_TESTS" = "1" ]; then ctest --output-on-failure -LE "long|performance"; fi && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


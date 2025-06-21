# Image: introlab3it/rtabmap:noble-kilted

FROM introlab3it/rtabmap:noble-kilted-deps

# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map && chmod 777 /root/Documents/RTAB-Map

# Copy current source code
COPY . /root/rtabmap

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DWITH_OPENGV=ON .. && \
    make -j4 && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


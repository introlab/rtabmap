# Image: introlab3it/rtabmap:focal-foxy

FROM introlab3it/rtabmap:focal-foxy-deps

# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map && chmod 777 /root/Documents/RTAB-Map

# Copy current source code
COPY . /root/rtabmap

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DWITH_ALICE_VISION=ON -DWITH_OPENGV=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig


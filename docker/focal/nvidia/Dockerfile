FROM introlab3it/rtabmap:focal

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
# Will be used to read/store databases on host
RUN mkdir -p /root/Documents/RTAB-Map

# On Nvidia Jetpack, uncomment the following (https://github.com/introlab/rtabmap/issues/776):
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra

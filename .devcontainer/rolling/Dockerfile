
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
RUN apt update && \
    apt install software-properties-common -y && \
    add-apt-repository universe -y && \
    apt update && \
    apt install curl -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    apt install /tmp/ros2-apt-source.deb && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# Install build dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt-get install -y \
      git \
      wget \
      libtbb-dev \
      libproj-dev \
      libpcl-dev \
      liboctomap-dev \
      libfreenect-dev \
      ros-rolling-ros-base \
      ros-dev-tools \
      ros-rolling-cv-bridge \
      ros-rolling-image-geometry \
      ros-rolling-laser-geometry \
      ros-rolling-pcl-conversions \
      ros-rolling-rviz-common \
      ros-rolling-rviz-rendering \
      ros-rolling-rviz-default-plugins \
      ros-rolling-pcl-ros \
      ros-rolling-imu-filter-madgwick \
      ros-rolling-image-transport \
      ros-rolling-octomap-msgs \
      ros-rolling-libg2o \
      ros-rolling-gtsam \
      ros-rolling-libpointmatcher \
      ros-rolling-qt-gui-cpp \
      ros-rolling-diagnostic-updater && \
    apt-get clean && rm -rf /var/lib/apt/lists/

WORKDIR /root/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN echo -e '#!/bin/bash\nset -e\n\n# setup ros2 environment\nsource "/opt/ros/rolling/setup.bash" --\nexec "$@"' > /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# ros2 seems not sourcing by default its multi-arch folders
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/rolling/lib/x86_64-linux-gnu

# For devcontainer
# remove ubuntu user
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

RUN apt-get update && apt-get install -y sudo && \
    apt-get clean && rm -rf /var/lib/apt/lists/

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -ex && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -a -G sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /home/${USERNAME}/Documents/RTAB-Map && chown -R ${USERNAME} /home/${USERNAME}

RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc


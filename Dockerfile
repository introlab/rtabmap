FROM mitmrg/gtsam-opencv-pcl:latest
# Possibly useful unsure
ENV GIT_SSH_COMMAND 'ssh -i ~/.ssh/id_rsa -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no'

ARG USER_ID
RUN adduser --uid $USER_ID mrg --disabled-password --gecos="mrg"
RUN usermod -aG sudo mrg
RUN echo "mrg ALL=NOPASSWD: ALL" >> /etc/sudoers
USER mrg
WORKDIR /home/mrg


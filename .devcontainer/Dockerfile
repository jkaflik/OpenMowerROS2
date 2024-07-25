FROM ros:iron
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout openmower

RUN apt-get update \
  && apt-get install -y ssh \
    python3-pip \
    curl \
    rsync \
    psmisc \
  && rm -rf /var/lib/apt/lists/*

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL /bin/bash

RUN mkdir -p /home/ws/build /home/ws/install /home/ws/log \
  && chown -R $USERNAME:$USERNAME /home/ws

RUN ( \
    echo "source /opt/ros/iron/setup.bash"; \
    echo "source /home/ws/install/setup.bash"; \
    echo "source /home/ws/.devcontainer/openmower_config.env"; \
    echo "export DISPLAY=:0"; \
  ) >> /home/$USERNAME/.bashrc

RUN ( \
    echo "PasswordAuthentication yes"; \
    echo "PermitRootLogin yes"; \
) >> /etc/ssh/sshd_config

RUN sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config
RUN mkdir /run/sshd

COPY entrypoint.sh /entrypoint.sh
RUN chmod +rx /entrypoint.sh

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/usr/bin/sudo", "/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config"]

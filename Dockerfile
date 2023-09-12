# At the moment it's completely based on .devcontainer/Dockerfile

FROM ros:iron
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG SSH_PORT=2222

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout openmower

RUN apt-get update \
  && apt-get install -y ssh \
    python3-pip \
    curl \
    vim \
  && rm -rf /var/lib/apt/lists/*

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL /bin/bash

RUN mkdir -p /home/ws/build /home/ws/install /home/ws/log \
  && chown -R $USERNAME:$USERNAME /home/ws

RUN ( \
    echo "source /opt/ros/iron/setup.bash"; \
    echo "source /home/ws/install/setup.bash"; \
  ) >> /home/$USERNAME/.bashrc

# Extra steps for having prebuild packages

USER $USERNAME
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /home/ws/src
COPY utils/ /home/ws/utils
COPY custom_deps.yaml /home/ws/custom_deps.yaml
COPY Makefile /home/ws/Makefile
COPY src/openmower/ /home/ws/src/openmower/
RUN source /home/$USERNAME/.bashrc \
    && cd /home/ws \
    && sudo apt-get update \
    && rosdep update \
    && make custom-deps deps

RUN source /opt/ros/iron/setup.bash \
    && cd /home/ws \
    && make build

CMD ["bash", "-c", "ros2 launch openmower openmower.launch.py"]

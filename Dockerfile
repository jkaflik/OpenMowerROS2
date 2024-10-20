FROM ros:jazzy
ARG USERNAME=openmower
ARG USER_UID=1001
ARG USER_GID=$USER_UID
ARG WORKSPACE=/opt/ws

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME

RUN ( \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
    echo "source ${WORKSPACE}/install/local_setup.bash"; \
  ) >> /home/$USERNAME/.bashrc

SHELL ["/bin/bash", "-c"]

RUN mkdir -p $WORKSPACE
COPY . $WORKSPACE

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $WORKSPACE \
    && sudo apt-get update \
    && rosdep update \
    && make custom-deps deps

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd $WORKSPACE && \
    make build-libs build

RUN chown -R $USERNAME:$USERNAME $WORKSPACE

USER $USERNAME
WORKDIR $WORKSPACE
ENV SHELL /bin/bash

CMD ["bash", "-c", "ros2 launch openmower openmower.launch.py"]

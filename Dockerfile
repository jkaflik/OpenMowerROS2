# This Dockerfile is itent to be a release image, but for now it is used for development

FROM ros:iron
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

RUN sudo apt install -y wget curl vim
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y gz-garden ros-iron-teleop-twist-keyboard
RUN sudo adduser $USERNAME dialout

COPY .devcontainer/entrypoint.sh /entrypoint.sh

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

COPY ./ /home/ws
WORKDIR /home/ws

RUN sudo chown -R openmower /home/ws/
RUN rosdep update
RUN make custom-deps deps build

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["ros2", "launch", "src/openmower/launch/openmower.launch.py"]

FROM ros:iron
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME

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

USER $USERNAME
CMD ["/bin/bash"]

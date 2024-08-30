FROM ros:jazzy
ARG USERNAME=dev
ARG USER_UID=1001
ARG USER_GID=$USER_UID

ENV DISPLAY=:0

RUN apt-get update && apt-get install --no-install-recommends -y \
  xfonts-base \
  xauth \
  x11-xkb-utils \
  xkb-data \
  dbus-x11 \
  tigervnc-standalone-server

COPY scripts/entrypoint.sh /entrypoint.sh

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL /bin/bash

USER $USERNAME

EXPOSE 5900

CMD ["/entrypoint.sh"]

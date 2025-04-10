# Build stage
FROM ros:jazzy AS builder
ARG WORKSPACE=/opt/ws

SHELL ["/bin/bash", "-c"]

# Install dependencies first to leverage caching
RUN mkdir -p $WORKSPACE
WORKDIR $WORKSPACE

# Copy only files needed for dependency installation first
COPY Makefile $WORKSPACE/
COPY package.xml $WORKSPACE/

# Install dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep update \
    && make custom-deps deps \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Now copy the rest of the source code
COPY . $WORKSPACE/

# Build the project
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && make build-libs build

# Runtime stage
FROM ros:jazzy
ARG USERNAME=openmower
ARG USER_UID=1001
ARG USER_GID=$USER_UID
ARG WORKSPACE=/opt/ws

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME

COPY --from=builder $WORKSPACE/package.xml $WORKSPACE/

# Install only runtime dependencies
RUN apt-get update \
    && rosdep update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $WORKSPACE \
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . \
    --ignore-src --skip-keys="$(rospack list-names | paste -s -d ' ' -)" \
    -r -y \
    --dependency-types=exec \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

COPY utils/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Copy build artifacts from builder stage
COPY --from=builder $WORKSPACE/install $WORKSPACE/install
COPY --from=builder $WORKSPACE/share $WORKSPACE/share

RUN mkdir -p $WORKSPACE \
    && chown -R $USERNAME:$USERNAME $WORKSPACE

USER $USERNAME
WORKDIR $WORKSPACE
ENV SHELL /bin/bash

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "openmower", "openmower.launch.py"]

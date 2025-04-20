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
COPY custom_deps.yaml $WORKSPACE/
COPY utils/install-custom-deps.sh $WORKSPACE/utils/

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
    && make build-release

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

# Switch to bash to ensure sourceing works
SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

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

# Copy build artifacts from builder stage
COPY --from=builder $WORKSPACE/install $WORKSPACE/install
COPY --from=builder $WORKSPACE/launch $WORKSPACE/launch
COPY --from=builder $WORKSPACE/config $WORKSPACE/config
COPY --from=builder $WORKSPACE/build $WORKSPACE/build
COPY --from=builder $WORKSPACE/description $WORKSPACE/description
COPY --from=builder /opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO

# Copy XML plugins definition
# TODO: this should be worked out better
COPY --from=builder $WORKSPACE/src/docking_helper/plugins.xml $WORKSPACE/src/docking_helper/plugins.xml

RUN mkdir -p $WORKSPACE \
    && chown -R $USERNAME:$USERNAME $WORKSPACE

COPY utils/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

USER $USERNAME
WORKDIR $WORKSPACE
ENV WORKSPACE=$WORKSPACE

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "open_mower_next", "openmower.launch.py"]

# Almost Devcontainer in CLion

## Overview

Devcontainer plugin in CLion is not as good as in VSCode. It's almost entirely broken.
An alternative is to use Docker Compose to run the container and configure CLion to use it as a remote toolchain.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [CLion](https://www.jetbrains.com/clion/download/) + extra [Docker plugin](https://plugins.jetbrains.com/plugin/7724-docker)

## Getting Started

1. **Open repository in CLion**.
2. **Run containers with Docker Compose**.
    ```bash
    cd .devcontainer/
    docker-compose up -d
    ```
3. **Configure CLion**.
      1. Open `Settings` -> `Build, Execution, Deployment` -> `Toolchains`.
      2. Add new toolchain with remote host `localhost`, port `2222` and remote user `openmower` authenticated with password `openmower`. 
      3. Inside workspace container run a command that retrieves ROS workspace env vars:
      ```bash
      docker compose exec workspace /home/ws/envs.sh
      ```
      4. In `Settings` -> `Build, Execution, Deployment` -> `CMake` pick your new toolchain and fill in `Environment` with env vars from previous step.
4. **Enjoy your development environment**.

::: tip
Please see [detailed information on containers](devcontainer#detailed).
:::

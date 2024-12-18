# Devcontainer

## Overview

Devcontainer is a recommended way to setup development environment for this project. It is a Docker container with all the required tools and dependencies.

With a seamless integration with VSCode, it provides a consistent development environment for all developers.

For CLion users there is an alternative approach described in [CLion development environment](./clion-env).

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [VSCode](https://code.visualstudio.com/download) or any other IDE/workspace that supports [Devcontainer](https://code.visualstudio.com/docs/remote/containers)
- [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for VSCode

## Getting Started with VSCode

1. **Open repository in VSCode**.
2. **Reopen in Container**. (VSCode should ask for it automatically, but if not, you can do it manually)
    - Click on the green icon in the bottom left corner of the VSCode window.
    - Select `Reopen in Container`.
    - Wait for the container to build.
    - VSCode will reopen in the container.
3. **Enjoy your development environment.**

## Getting Started with 

## Detailed

Devcontainer comes up with some containers configured with Docker Compose:
- `workspace` - main container with all the tools and dependencies and mounted workspace
- `xserver` - container with X server and VNC server for GUI applications
- `groot` - container with groot, a GUI for [BehaviourTree.CPP](https://www.behaviortree.dev/). It does not start by default. See [GRoot](./groot.md) for more details.

All containers share the same X server socket, so GUI applications can be run from the `workspace` container and displayed in the `xserver` container.
VNC server in `xserver` runs a web server on port `12345` with a VNC client. You can access it by opening [`http://localhost:12345`](http://localhost:12345) in your browser.

## Default environment variables loaded in the image

<<< ../.devcontainer/openmower_config.env{bash}

For more details about the environment variables, see [Configuration](configuration.md).

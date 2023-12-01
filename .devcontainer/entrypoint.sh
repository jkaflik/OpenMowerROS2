#!/usr/bin/env bash
set -e

if [ ! -d "/home/ws" ]; then
    echo "ERROR: /home/ws does not exist. Please mount your workspace to /home/ws"
    exit 1
fi

cd /home/ws

sudo apt update
rosdep update
make custom-deps deps

exec "$@"

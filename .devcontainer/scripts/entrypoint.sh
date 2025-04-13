#!/usr/bin/env bash

terminate_script() {
    echo "Termination signal received, exiting..."
    exit 1
}

trap terminate_script SIGINT

if [ -z "$DISPLAY" ]; then
    echo "DISPLAY not set, setting to :0"
    DISPLAY=:0
fi

RESOLUTION=${DESKTOP_RESOLUTION:-1920x1080}
echo "Using resolution: $RESOLUTION"

Xvnc -ac -pn -depth 24 -geometry $RESOLUTION $DISPLAY -rfbport=5900 -SecurityTypes=None -desktop=OpenMowerNext

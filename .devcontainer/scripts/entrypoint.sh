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

Xvnc -ac -pn -depth 24 $DISPLAY -rfbport=5900 -SecurityTypes=None -desktop=OpenMowerNext

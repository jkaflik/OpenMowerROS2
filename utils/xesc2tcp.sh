#!/bin/bash

PORT=1234

declare -A DEVICE_MAP=(
    ["left"]="/dev/ttyAMA4"
    ["right"]="/dev/ttyAMA2"
    ["mower"]="/dev/ttyAMA3"
)

if [[ -z $1 ]]; then
    echo "Error: Argument required."
    exit 1
fi

if [[ -z ${DEVICE_MAP[$1]} ]]; then
    echo "Error: Invalid argument. Valid values are: left, right, mower."
    exit 1
fi

DEVICE=${DEVICE_MAP[$1]}

running=true
trap 'echo "Interrupt received! Stopping..."; running=false' SIGINT

echo "Running socat for device: ${DEVICE} on port: ${PORT}..."

while $running; do
    sudo socat TCP-LISTEN:${PORT},reuseaddr,fork FILE:${DEVICE},b115200,cs8,raw,echo=0 || true
done

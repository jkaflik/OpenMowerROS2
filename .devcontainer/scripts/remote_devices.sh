#!/usr/bin/env bash

set -e

sudo apt-get install -y socat inetutils-ping psmisc > /dev/null
# This script is used to create a socat device for each serial port you want to use in the container

# get host from first argument
host=$1
username=${2:-openmower}
running=true

if [ -z "$host" ]
then
    echo "Host is empty"
    exit 1
fi

# check if RSA private key exists, if not create it
if [ ! -f ~/.ssh/id_rsa ]; then
    ssh-keygen -t rsa -b 4096 -C "devcontainer@devcontainer" -f ~/.ssh/id_rsa -q -N ""
fi

ssh-copy-id -i ~/.ssh/id_rsa.pub $username@$host

# check if can ssh to host
if ! ssh -o ConnectTimeout=2 $username@$host exit > /dev/null; then
    echo "Cannot connect to host"
    exit 1
fi

ssh $username@$host "sudo killall socat || true"

#trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

sshAndSocatSerialToTCP() {
    # $1 = serial port
    # $2 = baud rate
    # $3 = tcp port

    ssh -o ExitOnForwardFailure=yes $username@$host "true && sudo socat TCP-LISTEN:$3,reuseaddr,fork FILE:/dev/$1,b$2,cs8,raw,echo=0" & \
        sleep 1 && \
        sudo socat pty,link=/dev/$1,raw,group-late=dialout,mode=660 tcp:$host:$3 && fg
}

killall socat > /dev/null || true

sshAndSocatSerialToTCP ttyAMA0 115200 65000 & \
sshAndSocatSerialToTCP ttyAMA1 921600 65001 & \
sshAndSocatSerialToTCP ttyAMA2 115200 65002 & \
sshAndSocatSerialToTCP ttyAMA3 115200 65003 & \
sshAndSocatSerialToTCP ttyAMA4 115200 65004 && fg
&& fg

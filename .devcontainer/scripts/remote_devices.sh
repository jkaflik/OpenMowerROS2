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

isFirstRun=false

# check if RSA private key exists, if not create it
if [ ! -f ~/.ssh/id_rsa ]; then
    isFirstRun=true

    ssh-keygen -t rsa -b 4096 -C "devcontainer@devcontainer" -f ~/.ssh/id_rsa -q -N ""

    echo "Add the following public key to the authorized_keys file on the host:"
    echo "----------------------------------------"

    cat ~/.ssh/id_rsa.pub

    echo "----------------------------------------"
    echo "Press any key to continue"
    read -n 1 -s
fi

echo "Checking if host is reachable"

# check if can ssh to host
if ! ssh -o ConnectTimeout=2 $username@$host exit > /dev/null; then
    echo "Cannot connect to host"
    exit 1
fi

if [ "$isFirstRun" = true ]; then
    echo "Install socat on host"

    ssh $username@$host "sudo apt install -y socat > /dev/null"
fi

echo "Making sure all socat processes are killed on host"

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

echo "Starting socat processes"

sshAndSocatSerialToTCP ttyAMA0 115200 65000 & \
sshAndSocatSerialToTCP ttyAMA1 921600 65001 & \
sshAndSocatSerialToTCP ttyAMA2 115200 65002 & \
sshAndSocatSerialToTCP ttyAMA3 115200 65003 & \
sshAndSocatSerialToTCP ttyAMA4 115200 65004 && fg
&& fg

echo "All socat processes have exited, exiting..."

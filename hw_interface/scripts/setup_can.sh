#!/bin/bash

set -e

# load gs_usb
modprobe gs_usb

# default parameters
BITRATE=1000000
TXQUEUELEN=1000

# foreach can device
for CAN_DEVICE in $(ls /sys/class/net/ | grep robot_can); do
    echo "Configuring $CAN_DEVICE..."

    # set bitrate
    ip link set $CAN_DEVICE up type can bitrate $BITRATE
    if [ $? -eq 0 ]; then
        echo "Set bitrate to $BITRATE for $CAN_DEVICE."
    else
        echo "Failed to set bitrate for $CAN_DEVICE."
        continue
    fi

    # set tx queue length
    ip link set $CAN_DEVICE txqueuelen $TXQUEUELEN
    if [ $? -eq 0 ]; then
        echo "Set txqueuelen to $TXQUEUELEN for $CAN_DEVICE."
    else
        echo "Failed to set txqueuelen for $CAN_DEVICE."
        continue
    fi

    # setup can device
    ip link set up $CAN_DEVICE
    if [ $? -eq 0 ]; then
        echo "$CAN_DEVICE is up."
    else
        echo "Failed to bring up $CAN_DEVICE."
    fi
done

echo "All CAN devices configured."

#! /bin/bash

DEVICE_NAME=$1

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev)
do (
    syspath="${sysdevpath%dev}"
    devname="$(udevadm info -q name -p $syspath)"
    [[ "$devname" == "bus/"* ]] && exit
    eval "$(udevadm info -q property --export -p $syspath)"
    idserial="$(echo "$ID_SERIAL" | tr _ " ")"
    [[ "$idserial" != "$DEVICE_NAME"* ]] && exit
    echo $devname
) done

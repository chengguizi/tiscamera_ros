#!/bin/bash

# Reduce latency in the FTDI serial-USB kernel driver to 1ms
# This is required due to https://github.com/torvalds/linux/commit/c6dce262
for file in $(ls /sys/bus/usb-serial/devices/); do
  value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
  if [ $value -gt 1 ]; then
    echo "Setting low_latency mode for $file"
    sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
  fi
done

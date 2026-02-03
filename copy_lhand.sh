#!/bin/bash
set -e
SRC_DIR="/home/orangepi/Desktop/IntelliegntControllerHC/lhandpro_ws/src"
DEST_DIR="/home/orangepi/Desktop/IntelliegntControllerHC/ros2_ws/src"

echo "Copying from $SRC_DIR to $DEST_DIR"
cp -r "$SRC_DIR"/* "$DEST_DIR"/
echo "Copy done."

#!/bin/bash
cd ros2_ws
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

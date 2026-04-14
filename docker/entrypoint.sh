#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# Si el workspace ya está compilado, lo cargamos
if [ -f /root/rover_ws/install/setup.bash ]; then
    source /root/rover_ws/install/setup.bash
fi

exec "$@"

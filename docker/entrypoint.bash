#!/bin/bash
set -e

# Empty world dockerfile
if [ -z /home/colcon_ws/install/setup.bash ]; then
    source /home/colcon_ws/install/setup.bash
# tests dockerfile
else #if [ -z ~/lrauv_ws/install/setup.bash ]; then
    source /home/developer/lrauv_ws/install/setup.bash
    # Make sure we're in the right workingdir
    cd /home/developer/lrauv_ws
fi

exec "$@"



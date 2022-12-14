#!/bin/bash
set -eo pipefail

WORKSPACE_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $WORKSPACE_DIR

colcon build --merge-install --cmake-args -DBUILD_TESTING=ON

source install/setup.sh
colcon test --merge-install --event-handlers console_direct+
colcon test-result

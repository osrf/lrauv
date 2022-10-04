#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR=$( cd -- "$( dirname -- $(readlink "${BASH_SOURCE[0]}") )" &> /dev/null && pwd )
source $WORKSPACE_DIR/install/setup.sh
export PYTHONPATH=$WORKSPACE_DIR/install/lib/python

echo "$@"
exec "$@"

#!/usr/bin/env bash
set -eo pipefail

if [ $# -gt 1 ]; then
    echo "Usage: $0 [path/to/workspace]"
    exit 1
fi

if [ $# -ne 1 ]; then
    read -e -p 'Pick a workspace directory [.]:' WORKSPACE_DIR
    WORKSPACE_DIR=${WORKSPACE_DIR:-$(pwd)}
else
    WORKSPACE_DIR=$1
fi

mkdir -p $WORKSPACE_DIR/src

LRAUV_REPOSITORY_DIRNAME=$(dirname $(dirname $(dirname -- "${BASH_SOURCE[0]}")))
LRAUV_REPOSITORY_DIR=$( cd -- "$LRAUV_REPOSITORY_DIRNAME" &> /dev/null && pwd )
if git -C $LRAUV_REPOSITORY_DIR rev-parse --is-inside-work-tree 1>/dev/null 2>&1; then
    if [ "$(dirname $LRAUV_REPOSITORY_DIR)" != "$WORKSPACE_DIR/src" ]; then
        git clone $LRAUV_REPOSITORY_DIR $WORKSPACE_DIR/src/lrauv
        LRAUV_REPOSITORY_DIR=$WORKSPACE_DIR/src/lrauv
    fi
else
    LRAUV_REPOSITORY_DIR=$WORKSPACE_DIR/src/lrauv
    git clone https://github.com/osrf/lrauv.git $LRAUV_REPOSITORY_DIR
fi

cd $LRAUV_REPOSITORY_DIR
IMAGE_NAME="$(basename $WORKSPACE_DIR):latest"
docker build --target lrauv-base -t $IMAGE_NAME -f tools/setup/Dockerfile .
echo "$IMAGE_NAME" > $WORKSPACE_DIR/.image
cp -p tools/setup/enter-container.sh $WORKSPACE_DIR/enter

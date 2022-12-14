#!/bin/bash
set -euo pipefail

WORKSPACE_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$(xauth nlist $DISPLAY)")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_OPTS=
DOCKER_VERSION=$(docker version --format '{{.Server.Version}}')
if dpkg --compare-versions 19.03 gt "$DOCKER_VERSION"
then
    echo "Docker version is less than 19.03, using nvidia-docker2 runtime"
    if ! dpkg --list | grep nvidia-docker2
    then
        echo "Please either update docker-ce to a version greater than 19.03 or install nvidia-docker2"
	exit 1
    fi
    DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
fi

CONTAINER_IMAGE=${1:-$(cat $WORKSPACE_DIR/.image)}

CONTAINER_ID=$(docker ps -aqf "ancestor=${CONTAINER_IMAGE}")
if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_NAME="$(basename $WORKSPACE_DIR)_$(date +%s)"
    docker run --rm --privileged --net=host -u $(id -u):$(id -g) \
           --name $CONTAINER_NAME --security-opt seccomp=unconfined \
           -e DISPLAY -e MESA_GL_VERSION_OVERRIDE=3.3 -e XAUTHORITY=$XAUTH \
           $DOCKER_OPTS -e PULSE_SERVER=unix:$XDG_RUNTIME_DIR/pulse/native \
           -v $XDG_RUNTIME_DIR/pulse/native:$XDG_RUNTIME_DIR/pulse/native \
           -v $HOME/.config/pulse/cookie:$HOME/.config/pulse/cookie \
           -v "$XAUTH:$XAUTH" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v $WORKSPACE_DIR:/home/developer/lrauv_ws -it $CONTAINER_IMAGE bash
    CONTAINER_ID=$(docker ps -q --filter "name=$CONTAINER_NAME")
    if [ ! -z "$CONTAINER_ID" ]; then
       docker commit $CONTAINER_ID $CONTAINER_IMAGE
       docker stop $CONTAINER_ID
    fi
else
    docker exec --privileged -e DISPLAY -it $CONTAINER_ID bash
fi

#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [<dir with workspace> ...]"
    exit 1
fi

#IMG=$(basename $1)
IMG=$1
echo "Image: $1"

ARGS=("$@")
WORKSPACES=("${ARGS[@]:1}")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth

DOCKER_OPTS=

for WS_DIR in ${WORKSPACES[@]}
do
  WS_DIRNAME=$(basename $WS_DIR)
  DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:/home/developer/$WS_DIRNAME"
done

# development directory
#DOCKER_OPTS="$DOCKER_OPTS -v `pwd`/quadrotor-tunnel-nav:/home/developer/catkin_ws/src/quadrotor-tunnel-nav"

# Mount extra volumes if needed.
# E.g.:
# -v "/opt/sublime_text:/opt/sublime_text" \


docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -e NO_AT_BRIDGE=1 \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  --privileged \
  --rm \
  --security-opt seccomp=unconfined \
  $DOCKER_OPTS \
  $IMG

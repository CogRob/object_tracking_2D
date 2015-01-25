#!/bin/sh

version_gt() { test "$(echo "$@" | tr " " "\n" | sort -V | tail -n 1)" == "$1"; }
docker_version=$(docker version | grep 'Client version' | awk '{split($0,a,":"); print a[2]}' | tr -d ' ')
# Docker 1.3.0 or later is required for --device
if ! version_gt "${docker_version}" "1.2.0"; then
	echo "Docker version 1.3.0 or greater is required"
	exit 1
fi

if test $# -lt 1; then
	# Get the latest opengl-nvidia build
	# and start with an interactive terminal enabled
	args="-i -t $(docker images | grep ^cogrob/omnimapper-nvidia | head -n 1 | awk '{ print $1":"$2 }')"
else
        # Use this script with derived images, and pass your 'docker run' args
	args="$@"
fi


USER_UID=$(id -u)
USER_GID=$(id -g)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

DESKTOP="/home/${USER}/Desktop"
EBT_ROOT="$(dirname "$(pwd)")"

docker run \
	-it --rm \
	--volume=$DESKTOP:/home/dox/Desktop \
	--volume=$EBT_ROOT:/home/dox/git/object_tracking_2D_dev \
	--volume=/run/user/${USER_UID}/pulse:/run/pulse \
	--volume=$XSOCK:$XSOCK:rw \
	--volume=$XAUTH:$XAUTH:rw \
	--privileged -v /dev/video0:/dev/video0 \
	--device /dev/nvidia0:/dev/nvidia0 \
	--device /dev/nvidiactl:/dev/nvidiactl \
	--env="XAUTHORITY=${XAUTH}" \
	--env="USER_UID=${USER_UID}" \
	--env="USER_GID=${USER_GID}" \
	--env="DISPLAY=${DISPLAY}" \
	-u dox \
	$args
	
#--device=/dev/dri/card0:/dev/dri/card0 \

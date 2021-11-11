#!/bin/bash
#launch_simulation_toolbox.sh

BASEDIR=$(dirname "$0")
source "${BASEDIR}/useful_scripts.sh"

if [[ $OSTYPE == 'darwin'* ]]; then
	errmsg 'macOS is no longer supported. Please run this in an Ubuntu virtual machine.'
fi

exit_if_docker

if ! command -v docker &> /dev/null
then
	UTIL_LIST="docker.io build-essential cmake parallel"
	if command -v apt &> /dev/null
	then
		infomsg "Installing utilities for Debian/Ubuntu..."
		sudo apt-get update
		sudo apt-get install -y ${UTIL_LIST}
	elif command -v yum &> /dev/null
	then
		infomsg "Installing utilities for yum package manager..."
		sudo yum install -y ${UTIL_LIST}
	elif command -v snap &> /dev/null
	then
		infomsg "Installing utilities for snap..."
		sudo snap install -y ${UTIL_LIST}
	fi
fi

xhost + > /dev/null

export GID=$(id -g)
OS_NAME=$(uname -a)
XAUTH=/tmp/.docker.xauth
touch $XAUTH
XDISPL=`xauth nlist $DISPLAY`

if [ ! -z "$XDISPL" ]
then
  xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - > /dev/null
fi

chmod 777 $XAUTH

if [[ $OSTYPE == 'darwin'* ]]; then
  DISPLAY_CMD=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/host.docker.internal\1/'`
else
  DISPLAY_CMD=$DISPLAY
fi

OS_SPECIFIC_FLAGS=""

if [[ "$OS_NAME" == *"penguin"* ]]; then
	infomsg "Chrome OS detected"
else
	OS_SPECIFIC_FLAGS="--privileged --device=/dev/dri:/dev/dri"
fi

cp ~/.gitconfig $(pwd)

mkdir -p "$(pwd)/.parallel"
touch "$(pwd)/.parallel/will-cite"

docker pull guitar24t/ck-ros:latest || true
#clear terminal without destroying scrollback buffer
printf "\033[2J\033[0;0H"
docker run -ti --rm \
	   -e DISPLAY=$DISPLAY_CMD \
	   $OS_SPECIFIC_FLAGS \
	   -e XAUTHORITY=$XAUTH \
       	   -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
	   -v $(pwd):/mnt/working \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v ~/.ssh:/home/$USER/.ssh \
	   -v $XAUTH:$XAUTH \
	   --user $UID:$GID \
	   --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --net=host \
       -e HOME=/mnt/working \
	   guitar24t/ck-ros:latest \
	   /bin/bash --rcfile /mnt/.bashrc
	   

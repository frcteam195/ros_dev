#!/bin/bash
#launch_simulation_toolbox.sh

BASEDIR=$(dirname "$0")
source "${BASEDIR}/useful_scripts.sh"

if [[ $(pwd) == *"ros_dev"* ]]; then
  infomsg "This script cannot be run from this directory. Attempting to fix..."
  cd ..
  if [ -d "$(pwd)/$(ls | grep *_Robot)" ]; then
    infomsg "Correct directory found. Launching..."
  else
    errmsg "Unable to detect the proper directory to run from..."
    exit 1
  fi
fi

exit_if_macOS
exit_if_docker

DETACHED_MODE=
DOCKER_CMD_VAR=
FORCED_LAUNCH=
DOCKER_RUNNING_CMD=1

CONTAINER_ID=`docker ps -aq --filter "ancestor=guitar24t/ck-ros:latest" --filter "status=running"`

usage() { infomsg "Usage: $0 [-d] [-k] [-h] [-c <string>]\n\t-d Run docker container in detached mode\n\t-k Kill running docker instance\n\t-c <string> Run a command in the docker container\n\t-h Display this help text \n\n" 1>&2; exit 1; }
while getopts "fdkhc:" o; do
    case "${o}" in
        d)
			DETACHED_MODE=-d
            ;;
		f)
			FORCED_LAUNCH=0
			;;
		k)
			if [ ! -z "${CONTAINER_ID}" ] 
			then
				infomsg "Stopping container..."
				docker stop ${CONTAINER_ID}
				infomsg "Exiting..."
				exit 0;
			fi
			errmsg "No docker container found to kill!" noexit
			usage
			;;
        c)
            DOCKER_RUNNING_CMD=0
			DOCKER_CMD_VAR="${OPTARG}"
            ;;
        h | *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

if [ ! -z "${CONTAINER_ID}" ] && [ -z "${FORCED_LAUNCH}" ]
then
    infomsg "Docker container is already running! We will launch a new terminal to it instead..."
    infomsg "You can stop this container using ${0} -k"
	docker exec -it $CONTAINER_ID /bin/bash
	exit 0;
elif [ ! -z "${CONTAINER_ID}" ] && [ ! -z "${FORCED_LAUNCH}" ]
then
	infomsg "A docker instance is already running, but you have chosen to force launch a new instance. Hope you know what you're doing..."
fi

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
if [[ "${DOCKER_RUNNING_CMD}" -eq 1 ]];
then
	if [ ! -z "${DETACHED_MODE}" ];
	then
		infomsg "Launching a detached container of this docker instance:"
	else
		#clear terminal without destroying scrollback buffer
		printf "\033[2J\033[0;0H"
	fi

	docker run -it ${DETACHED_MODE} --rm \
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
	   /bin/bash
else
	docker run -it --rm \
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
	   /bin/bash -ci "${DOCKER_CMD_VAR}"
fi

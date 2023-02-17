#!/bin/bash
#################################################
#run_container.sh
#Version 2022.01.19
#Authors: FRC Team 195
#################################################


BASEDIR=$(dirname "${0}")
source "${BASEDIR}/useful_scripts.sh"
INET_ONLINE=$(timeout 0.5s ping -c1 8.8.8.8 > /dev/null; echo ${?})

if [ ! -f "${BASEDIR}/../.bash_completion" ]; then
	echo '#!/bin/bash' > "${BASEDIR}/../.bash_completion"
	echo -e "\n" >> "${BASEDIR}/../.bash_completion"
	cat "${BASEDIR}/bash_completion.sh" >> "${BASEDIR}/../.bash_completion"
	echo -e "\n" >> "${BASEDIR}/../.bash_completion"
	chmod +x "${BASEDIR}/../.bash_completion"
	infomsg "Installed bash completions"
else
	NEW_COMPLETIONS=$(<"${BASEDIR}/bash_completion.sh")
	CURR_COMPLETIONS=$(<"${BASEDIR}/../.bash_completion")
	if [[ ${CURR_COMPLETIONS} = *"${NEW_COMPLETIONS}"* ]];
	then
		infomsg "Bash completions already installed"
	else
		echo -e "\n" >> "${BASEDIR}/../.bash_completion"
		cat "${BASEDIR}/bash_completion.sh" >> "${BASEDIR}/../.bash_completion"
		echo -e "\n" >> "${BASEDIR}/../.bash_completion"
		infomsg "Bash completions installed to your already existing completion script"
	fi
fi

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

DETACHED_MODE=
DOCKER_CMD_VAR=
FORCED_LAUNCH=
DOCKER_RUNNING_CMD=1
DOCKER_ARCH=latest

CONTAINER_ID=$(docker ps -aql --filter "ancestor=guitar24t/ck-ros:${DOCKER_ARCH}" --filter "status=running")

usage() { infomsg "Usage: ${0} [-a] [-d] [-k] [-h] [-c <string>]\n\t-a Force arm64 docker container\n\t-i Force x86_64 docker container\n\t-d Run docker container in detached mode\n\t-k Kill running docker instance\n\t-c <string> Run a command in the docker container\n\t-h Display this help text \n\n" 1>&2; exit 1; }
while getopts "ac:dfhik" o; do
    case "${o}" in
		a)
			DOCKER_ARCH=arm64
			if [ ${INET_ONLINE} -eq 0 ]; then
				docker pull guitar24t/ck-ros:arm64 || true
			fi
			;;
        d)
			DETACHED_MODE=-d
            ;;
		f)
			FORCED_LAUNCH=0
			;;
		i)	
			DOCKER_ARCH=amd64
			if [ ${INET_ONLINE} -eq 0 ]; then
				docker pull guitar24t/ck-ros:amd64 || true
			fi
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
			FORCED_LAUNCH=0
			DOCKER_CMD_VAR="${OPTARG}"
			DETACHED_MODE=-d
            ;;
        h | *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

CONTAINER_ID=$(docker ps -aql --filter "ancestor=guitar24t/ck-ros:${DOCKER_ARCH}" --filter "status=running")

#TODO: Figure out why we did this
#if [ ${DOCKER_ARCH} == "latest" ] && [ ! -z ${FORCED_LAUNCH} ]
#then 
#	exit_if_docker
#fi

if [ ! -z "${CONTAINER_ID}" ] && [ -z "${FORCED_LAUNCH}" ]
then
    infomsg "Docker container is already running! We will launch a new terminal to it instead..."
    infomsg "You can stop this container using ${0} -k"
	docker exec -it ${CONTAINER_ID} /bin/bash
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
touch ${XAUTH}
XDISPL=$(xauth nlist ${DISPLAY})

if [ ! -z "${XDISPL}" ]
then
  xauth nlist ${DISPLAY} | sed -e 's/^..../ffff/' | xauth -f ${XAUTH} nmerge - > /dev/null
fi

chmod 777 ${XAUTH}

DISPLAY_FLAGS="-e DISPLAY=${DISPLAY}"
USER_FLAGS="-e USER=${USER} --user ${UID}:${GID}"
RENDERING_FLAGS="--device=/dev/dri:/dev/dri"
DCUDA_FLAGS=""
USER_HOME_MAPPING_FLAGS="-v $(realpath ${BASEDIR}/../.${USER}/):/home/${USER}"

#Running on macOS
if [[ ${OS_NAME} == *"Darwin"* ]]; then
	DISPLAY_CMD=`echo ${DISPLAY} | sed 's/^[^:]*\(.*\)/host.docker.internal\1/'`
fi

#Running on Chromebook
if [[ "${OS_NAME}" == *"penguin"* ]]; then
	infomsg "Chrome OS detected... Disabling rendering passthru!"
	RENDERING_FLAGS=""
fi

#Running on Jetson
if [ -f "/etc/nv_tegra_release" ]; then
	infomsg "Jetson device detected... Running as root in container!"
	USER_FLAGS=""
	USER_HOME_MAPPING_FLAGS=""
fi

#Running on CUDA Compatible System
if [ -d "/usr/local/cuda" ]; then
	infomsg "CUDA detected... Adding CUDA support to the container"
	DCUDA_FLAGS="--runtime nvidia"
fi

cp "${HOME}/.gitconfig" "$(pwd)"

mkdir -p "$(pwd)/.parallel"
touch "$(pwd)/.parallel/will-cite"

if [ "${USER_HOME_MAPPING_FLAGS}" != "" ]; then
	mkdir -p "${BASEDIR}/../.${USER}"
	cp -r "${HOME}/.ssh" "${BASEDIR}/../.${USER}/"
fi

if [ ${INET_ONLINE} -eq 0 ]; then
	infomsg "Checking for container updates..."
	docker pull guitar24t/ck-ros:${DOCKER_ARCH} || true
fi

if [ ! -z "${DETACHED_MODE}" ];
then
	infomsg "Launching a detached container of this docker instance:"
else
	#clear terminal without destroying scrollback buffer
	printf "\033[2J\033[0;0H"
fi

COMMAND_NEEDS_LAUNCH=1
if [[ "${DOCKER_RUNNING_CMD}" -eq 0 && ${CONTAINER_ID} == "" ]]; then
	COMMAND_NEEDS_LAUNCH=0
fi

if [[ "${DOCKER_RUNNING_CMD}" -eq 1 || "${COMMAND_NEEDS_LAUNCH}" -eq 0 ]]; then
	WKSP_DIR=$(pwd)
	mkdir -p "$WKSP_DIR/.vscode"
	CURR_VSCODE1_MD5=($(md5sum ${WKSP_DIR}/.vscode/c_cpp_properties.json))
	#CURR_VSCODE2_MD5=($(md5sum ${WKSP_DIR}/.vscode/settings.json))
	EXP_VSCODE1_MD5=($(md5sum ${WKSP_DIR}/ros_dev/vscode_workspace_config/c_cpp_properties.json))
	#EXP_VSCODE2_MD5=($(md5sum ${WKSP_DIR}/ros_dev/vscode_workspace_config/settings.json))
	#|| ${CURR_VSCODE2_MD5} != ${EXP_VSCODE2_MD5}
	if [[ ${CURR_VSCODE1_MD5} != ${EXP_VSCODE1_MD5} ]]; then
		echo "Your current cpp vscode config does not match expected. Press any key to replace config with proper config for workspace..."
		read -n 1 k <&1
	fi

	if [[ ! -f $WKSP_DIR/.vscode/settings.json ]]; then
		cp $(pwd)/ros_dev/vscode_workspace_config/settings.json $(pwd)/.vscode/settings.json
	fi

	rm -Rf $(pwd)/.vscode/c_cpp_properties.json
	cp $(pwd)/ros_dev/vscode_workspace_config/c_cpp_properties.json $(pwd)/.vscode/c_cpp_properties.json


	cd ./*trajectories_*
	if [ $? -eq 0 ]; then
		TRAJ_DIR=$(pwd)
		cd ..
		echo "Mapping Trajectories..."
		mkdir -p ./tmptraj
		rm -Rf ./tmptraj/**
		cp ${TRAJ_DIR}/**/*.json ./tmptraj/
		cp ${TRAJ_DIR}/*.json ./tmptraj/ 2>>/dev/null
		cp ${TRAJ_DIR}/**/*.shoe ./tmptraj/
		cp ${TRAJ_DIR}/*.shoe ./tmptraj/ 2>>/dev/null
	else
		echo "No trajectories found"
	fi

	TRAJ_CMD=
	if [[ "${TRAJ_DIR}" != 0 ]]; then
		TRAJ_CMD=--volume="$(pwd)/tmptraj:/robot/trajectories:ro"
	fi

	docker run -it ${DETACHED_MODE} --rm \
		${DISPLAY_FLAGS} \
		${RENDERING_FLAGS} \
		${USER_FLAGS} \
		--ipc="host" \
		-e XAUTHORITY=${XAUTH} \
		-v $(pwd):/mnt/working \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		${USER_HOME_MAPPING_FLAGS} \
		-v ${XAUTH}:${XAUTH} \
		${DCUDA_FLAGS} \
		--privileged \
		--volume="/etc/group:/etc/group:ro" \
		--volume="/etc/gshadow:/etc/gshadow:ro" \
		--volume="/etc/passwd:/etc/passwd:ro" \
		--volume="/etc/shadow:/etc/shadow:ro" \
		${TRAJ_CMD} \
		--net=host \
		-e HOME=/mnt/working \
		guitar24t/ck-ros:${DOCKER_ARCH} \
		/bin/bash

	CONTAINER_ID=$(docker ps -aql --filter "ancestor=guitar24t/ck-ros:latest" --filter "status=running")
fi

if [[ "${DOCKER_RUNNING_CMD}" -eq 0 ]]; then
	infomsg "Running command in container..."
	docker exec -it ${CONTAINER_ID} /bin/bash -ci "${DOCKER_CMD_VAR}" || true
	if [[ "${COMMAND_NEEDS_LAUNCH}" -eq 0 ]]; then
		infomsg "Stopping temporary container..."
		docker stop ${CONTAINER_ID} > /dev/null
	fi
	infomsg "Done!"
fi

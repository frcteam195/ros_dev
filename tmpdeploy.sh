#!/bin/bash

TARGET_IP=10.1.95.4

	if [ ! $# -eq 0 ]
	then
        TARGET_IP=${1}
	fi

BASEDIR=$(dirname "$0")
ROOT_DIR=$(realpath $(dirname ${BASEDIR}))
source "${BASEDIR}/useful_scripts.sh"
OS_ARCHITECTURE=$(arch)

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

#ROSLIB_PATH="outputs/aarch64/devel/lib"
#if [ $OS_ARCHITECTURE == 'aarch64' ] || [ $OS_ARCHITECTURE == 'arm64' ]
#then
#    ROSLIB_PATH="outputs/native/devel/lib"
#fi

BASE_PATH=$(find . -maxdepth 1 -type d -name '*_Robot*' -print -quit | xargs realpath -P)
#FULL_ROSLIB_PATH="${BASE_PATH}/${ROSLIB_PATH}"
#cd ${FULL_ROSLIB_PATH}
cd ${BASE_PATH}/..
echo "Packing robot..."
tar -hczf ${ROOT_DIR}/rosdeploy.tar.gz *_Robot/*
cd  ${ROOT_DIR}
echo "Deploying robot to target..."
scp rosdeploy.tar.gz  team195@${TARGET_IP}:/robot
echo "Unpacking robot on target..."
ssh team195@${TARGET_IP} '/robot/robot_scripts/unpackros.sh'
echo "Done!"
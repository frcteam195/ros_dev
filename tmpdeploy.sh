#!/bin/bash

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

ROSLIB_PATH="outputs/aarch64/devel/lib"
if [ $OS_ARCHITECTURE == 'aarch64' ] || [ $OS_ARCHITECTURE == 'arm64' ]
then
    ROSLIB_PATH="outputs/native/devel/lib"
fi

BASE_PATH=$(dirname $(find . -maxdepth 2 -type d -name '*catkin_ws*' -print -quit | xargs realpath -P))

FULL_ROSLIB_PATH="${BASE_PATH}/${ROSLIB_PATH}"
cd ${FULL_ROSLIB_PATH}
tar -czvf ${ROOT_DIR}/rosdeploy.tar.gz ./*_node
cd  ${ROOT_DIR}
scp rosdeploy.tar.gz  rhilton@10.1.95.5:/home/rhilton/
ssh rhilton@10.1.95.5 '/home/rhilton/unpackros.sh'

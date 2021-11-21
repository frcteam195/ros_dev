#!/bin/bash

BASEDIR=$(dirname "$0")
source "${BASEDIR}/useful_scripts.sh"

exit_if_macOS
exit_if_docker

CONTAINER_ID=`docker ps -aq --filter "ancestor=guitar24t/ck-ros:latest" --filter "status=running"`

if [ -z "${CONTAINER_ID}" ] 
then
    errmsg "No running ck-ros docker instance could be detected! You must first run the launch script!"
fi

docker exec -it $CONTAINER_ID /bin/bash

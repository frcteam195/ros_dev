#!/bin/bash
CONTAINER_ID=`docker ps -aq --filter "ancestor=guitar24t/ck-ros:latest" --filter "status=running"`

if [ -z "${CONTAINER_ID}" ] 
then
    echo "No running ck-ros docker instance could be detected! You must first run the launch script!"
    exit 1
fi

docker exec -it $CONTAINER_ID /bin/bash --rcfile /mnt/.bashrc

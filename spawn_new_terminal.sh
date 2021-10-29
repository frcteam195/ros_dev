#!/bin/bash
CONTAINER_ID=`docker ps -aq --filter "ancestor=guitar24t/ck-ros-dev:latest" --filter "status=running"`
docker exec -it $CONTAINER_ID /bin/bash --rcfile /mnt/.bashrc

#!/bin/bash

currdatestr=$(date +%mx%dx%Y)
currdate=$(date +%s)

push ()
{
 docker push guitar24t/ck-ros:arm64
}

tarball ()
{
  docker ps -aq | xargs docker rm -fv || true
  docker run -d -it --rm guitar24t/ck-ros:arm64 bash
  sleep 2
  CONTAINER_ID=$(docker ps -aq --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running")
  CONTAINER_NAME=$(docker ps -a --format '{{.Names}}' --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running")
  docker export ${CONTAINER_NAME} | gzip > ck_ros_arm64_${currdatestr}.tar.gz
  docker stop ${CONTAINER_ID}
  docker ps -aq | xargs docker rm -fv || true
}

build ()
{
  docker build -t guitar24t/ck-ros:arm64 --build-arg ARCH=arm64/ --build-arg NOW=${currdate} .
}

case "$1" in
  "push")
    push
    ;;
  "tarball")
    tarball
    ;;
  *)
    build
    ;;
esac

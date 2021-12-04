currdatestr := $(shell date +%mx%dx%Y)
currdate := $(shell date +%s)

CONTAINER_ID := $(shell docker ps -aq --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running")
CONTAINER_NAME := $(shell docker ps -a --format '{{.Names}}' --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running")

docker:
	docker build -t guitar24t/ck-ros:arm64 --build-arg ARCH=arm64/ --build-arg NOW=$(currdate) .

dockerpush:
	docker push guitar24t/ck-ros:arm64

dockertarball:
	docker run -d -it --rm guitar24t/ck-ros:arm64 bash
	docker export $(CONTAINER_NAME) | gzip > ck_ros_arm64_$(currdatestr).tar.gz
	docker stop ${CONTAINER_ID}


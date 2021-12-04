currdatestr := $(shell date +%mx%dx%Y)
currdate := $(shell date +%s)

docker:
	docker build -t guitar24t/ck-ros:arm64 --build-arg ARCH=arm64/ --build-arg NOW=$(currdate) .

dockerpush:
	docker push guitar24t/ck-ros:arm64

dockertarball:
	docker ps -aq | xargs docker rm -fv || true
	docker run -d -it --rm guitar24t/ck-ros:arm64 bash
	sleep 2
	docker export $(shell docker ps -a --format '{{.Names}}' --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running") | gzip > ck_ros_arm64_$(currdatestr).tar.gz
	docker stop $(shell docker ps -aq --filter "ancestor=guitar24t/ck-ros:arm64" --filter "status=running")
	docker ps -aq | xargs docker rm -fv


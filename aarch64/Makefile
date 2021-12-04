docker:
	docker build -t guitar24t/ck-ros:arm64 --build-arg ARCH=arm64/ --build-arg NOW=$(date +%s) .

dockerpush:
	docker push guitar24t/ck-ros:arm64

dockertarball:
	docker save guitar24t/ck-ros:arm64 | gzip > "ck_ros_arm64_$(date +%mx%dx%Y).tar.gz"



currdatestr := $(shell date +%mx%dx%Y)
currdate := $(shell date +%s)

docker:
	docker build -t guitar24t/ck-ros:amd64 --build-arg ARCH=amd64/ --build-arg NOW=$(currdate) .

dockerpush:
	docker push guitar24t/ck-ros:amd64

dockermanifest:
	docker pull guitar24t/ck-ros:arm64
	docker manifest rm guitar24t/ck-ros || true
	docker manifest create guitar24t/ck-ros:latest guitar24t/ck-ros:amd64 guitar24t/ck-ros:arm64
	docker manifest push guitar24t/ck-ros:latest

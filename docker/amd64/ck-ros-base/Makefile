currdatestr := $(shell date +%mx%dx%Y)
currdate := $(shell date +%s)
imagename := ck-ros-base
architecture := amd64
oppositearchitecture := arm64

docker:
	docker build -t guitar24t/$(imagename):$(architecture) --build-arg ARCH=$(architecture)/ --build-arg NOW=$(currdate) .

dockerpush:
	docker push guitar24t/$(imagename):$(architecture)

dockermanifest:
	docker pull guitar24t/$(imagename):$(oppositearchitecture)
	docker manifest rm guitar24t/$(imagename) || true
	docker manifest create guitar24t/$(imagename):latest guitar24t/$(imagename):$(architecture) guitar24t/$(imagename):$(oppositearchitecture)
	docker manifest push guitar24t/$(imagename):latest

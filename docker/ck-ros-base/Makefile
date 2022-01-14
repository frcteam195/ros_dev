currdatestr := $(shell date +%mx%dx%Y)
currdate := $(shell date +%s)
imagename := ck-ros-base
armarchitecture := arm64
x64architecture := amd64

dockerarm:
	docker build -t guitar24t/$(imagename):$(armarchitecture) --build-arg ARCH=$(armarchitecture) --build-arg NOW=$(currdate) .

dockerx64:
	docker build -t guitar24t/$(imagename):$(x64architecture) --build-arg ARCH=$(x64architecture) --build-arg NOW=$(currdate) .

dockerpusharm:
	docker push guitar24t/$(imagename):$(armarchitecture)

dockerpushx64:
	docker push guitar24t/$(imagename):$(x64architecture)

dockermanifeston:
	docker pull guitar24t/$(imagename):$(armarchitecture)
	docker pull guitar24t/$(imagename):$(x64architecture)
	docker manifest rm guitar24t/$(imagename) || true
	docker manifest create guitar24t/$(imagename):latest guitar24t/$(imagename):$(x64architecture) guitar24t/$(imagename):$(armarchitecture)
	docker manifest push guitar24t/$(imagename):latest

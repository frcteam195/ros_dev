# ros_dev

For Docker multiarch:
```
docker build -t guitar24t/ck-ros:amd64 --build-arg ARCH=amd64/
docker push guitar24t/ck-ros:amd64

docker build -t guitar24t/ck-ros:arm64 --build-arg ARCH=arm64/
docker push guitar24t/ck-ros:arm64

docker manifest create guitar24t/ck-ros:latest --amend guitar24t/ck-ros:amd64 --amend guitar24t/ck-ros:arm64
docker manifest push guitar24t/ck-ros:latest
```

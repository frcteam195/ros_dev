FROM stereolabs/zed:3.6-devel-cuda10.2-ubuntu18.04
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /tmp

RUN apt-get update && apt-get upgrade -y && apt-get install -y gnupg cmake build-essential apt-utils wget software-properties-common x11-apps net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules libboost-all-dev
RUN git clone https://github.com/IntelRealSense/librealsense \
        && cd librealsense \
	&& git checkout c94410a420b74e5fb6a414bd12215c05ddd82b69 \
        && chmod +x ./scripts/* \
        && apt-get update && apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev unzip libglfw3-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev -y \
        && ./scripts/setup_udev_rules.sh \
        && mkdir build \
        && cd build \
        && cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=false \
        && make -j$(($(nproc)-1)) \
        && make install \
	&& cd / \
        && rm -Rf /tmp/* \
        && ldconfig

WORKDIR /

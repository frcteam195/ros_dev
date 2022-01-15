FROM stereolabs/zed:3.6-devel-jetson-jp4.6
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /tmp
RUN apt-get update && apt-get upgrade -y && apt-get install -y gnupg apt-utils wget software-properties-common x11-apps net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules libboost-all-dev
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
        && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
        && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
        && apt-get install -y librealsense2-utils librealsense2-dev \
        && git clone https://github.com/IntelRealSense/librealsense \
        && cd librealsense \
	&& git checkout c94410a420b74e5fb6a414bd12215c05ddd82b69 \
        && chmod +x ./scripts/* \
        && apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y \
        && ./scripts/setup_udev_rules.sh \
        && mkdir build \
        && cd build \
        && cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true \
        && make -j$(($(nproc)-1)) \
        && make install \
	&& cd / \
        && rm -Rf /tmp/* \
        && ldconfig

WORKDIR /

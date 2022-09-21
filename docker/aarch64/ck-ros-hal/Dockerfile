FROM nvcr.io/nvidia/l4t-base:r35.1.0
LABEL Name="ck-ros-hal-aarch64"
LABEL Version="35.1.0"
USER root
ARG DEBIAN_FRONTEND=noninteractive
ARG GITLFSVER=2.30.0
WORKDIR /tmp
RUN apt-get update \
	&& apt-get install -y apt-utils \
	&& apt-get upgrade -y \
	&& apt-get install -y software-properties-common wget dh-autoreconf libcurl4-gnutls-dev libexpat1-dev gettext libz-dev libssl-dev \
		asciidoc xmlto docbook2x install-info \
	&& cd /tmp \
	&& rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update \
	&& apt-get install -y gnupg cmake build-essential wget software-properties-common x11-apps net-tools iputils-ping vim emacs extra-cmake-modules libboost-all-dev \
	&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev unzip sudo -y \
	&& rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:git-core/ppa \
	&& apt-get update \
	&& apt-get install git \
	&& cd /tmp \
	&& mkdir gitlfs \
	&& cd /tmp/gitlfs \
	&& wget https://github.com/git-lfs/git-lfs/releases/download/v${GITLFSVER}/git-lfs-linux-arm64-v${GITLFSVER}.tar.gz \
	&& tar -xzvf git-lfs-linux-arm64-v${GITLFSVER}.tar.gz \
	&& PREFIX=/usr ./install.sh \
	&& cd /tmp \
	&& apt-mark hold git-lfs \
	&& rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*

# RUN git clone https://github.com/IntelRealSense/librealsense \
# 	&& cd librealsense \
# 	&& git checkout c94410a420b74e5fb6a414bd12215c05ddd82b69 \
# 	&& chmod +x ./scripts/* \
# 	&& cp ./config/99-realsense-libusb.rules /lib/udev/rules.d/ \
# 	&& mkdir build \
# 	&& cd build \
# 	&& cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=true \
# 	&& make -j$(($(nproc)-1)) \
# 	&& make install \
# 	&& cd / \
# 	&& rm -Rf /tmp/* \
# 	&& rm -rf /var/lib/apt/lists/* \
# 	&& ldconfig

# RUN udevadm control --reload-rules; udevadm trigger

WORKDIR /
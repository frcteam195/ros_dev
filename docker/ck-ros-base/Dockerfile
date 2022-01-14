ARG ARCH
FROM guitar24t/ck-ros-hal:${ARCH}
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && apt-get install -y apt-utils wget software-properties-common x11-apps net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules libboost-all-dev python-pip bash-completion nano parallel

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list \
	&& apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
	&& apt-key list \
	&& apt-get update \
	&& apt-get -y install ros-melodic-desktop-full libsdl-dev libsdl-image1.2-dev libsuitesparse-dev ros-melodic-libg2o docker.io qemu libglfw3 libglfw3-dev

RUN echo 'root:robots' | chpasswd

RUN mkdir /mnt/working
WORKDIR /mnt/working

RUN pip install -U rosdep && rosdep init && rosdep update && pip install -U rosinstall vcstools rospkg

WORKDIR /tmp
RUN mkdir dummy \
	&& cd /tmp/dummy \
	&& git init \
	&& git lfs install \
	&& rm -Rf /tmp/*

WORKDIR /tmp
RUN git clone https://github.com/stiffstream/sobjectizer \
	&& cd /tmp/sobjectizer \
	&& git checkout 972b5310b7a486dd4d4322ffb46f1c7e15c47ef6 \
	&& mkdir cmake_build \
	&& cd /tmp/sobjectizer/cmake_build \
	&& cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ../dev \
	&& cmake --build . --config Release \
	&& cmake --build . --config Release --target install \
	&& rm -Rf /tmp/*

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSlibzmq.git \
	&& cd /tmp/CKROSlibzmq \
	&& git checkout 4097855ddaaa65ed7b5e8cb86d143842a594eebd \
	&& mkdir cppbuild \
	&& cd /tmp/CKROSlibzmq/cppbuild \
	&& cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	&& make -j8 \
	&& make install \
	&& rm -Rf /tmp/*

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSprotobuf.git \
	&& cd /tmp/CKROSprotobuf \
	&& git checkout 89b14b1d16eba4d44af43256fc45b24a6a348557 \
	&& mkdir cppbuild \
	&& cd /tmp/CKROSprotobuf/cppbuild \
	&& cmake ../cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	&& make -j8 \
	&& make install \
	&& rm -Rf /tmp/*

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSfmt.git \
	&& cd /tmp/CKROSfmt \
	&& git checkout d141cdbeb0fb422a3fb7173b285fd38e0d1772dc \
	&& mkdir cppbuild \
	&& cd /tmp/CKROSfmt/cppbuild \
	&& cmake .. \
	&& make -j8 \
	&& make install \
	&& rm -Rf /tmp/*

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSSDL.git \
	&& cd /tmp/CKROSSDL \
	&& git checkout 2e9821423a237a1206e3c09020778faacfe430be \
	&& ./configure \
	&& make -j8 \
	&& make install \
	&& rm -Rf /tmp/*

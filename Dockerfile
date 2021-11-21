FROM ubuntu:18.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y apt-utils
RUN apt-get install -y wget
RUN apt-get install -y software-properties-common

RUN apt-get install -y x11-apps

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key list
RUN apt-get update
RUN apt-get -y install ros-melodic-desktop-full

RUN apt-get -y install net-tools
RUN apt-get -y install iputils-ping

RUN echo 'root:robots' | chpasswd

RUN apt-get install -y vim
RUN apt-get install -y emacs

RUN apt-get install -y git
RUN apt-get install -y extra-cmake-modules

RUN mkdir /mnt/working
WORKDIR /mnt/working

RUN apt-get -y install libboost-all-dev
RUN apt-get -y install python-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN pip install -U rosinstall vcstools rospkg

WORKDIR /tmp
RUN curl -L https://developer.nvidia.com/embedded/dlc/l4t-gcc-7-3-1-toolchain-64-bit -o toolchain.tar.xz
RUN unxz toolchain.tar.xz
RUN tar -xvf toolchain.tar
RUN mv gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu /jetsontoolchain
RUN rm -Rf toolchain.tar

RUN apt-get install git-lfs
RUN mkdir dummy
WORKDIR /tmp/dummy
RUN git init
RUN git lfs install
WORKDIR /tmp
RUN git clone https://github.com/frcteam195/ck_jetson_run_tar.git
WORKDIR /tmp/ck_jetson_run_tar
RUN mkdir -p /jetsonfs
RUN tar -xzvf ck_jetson_run_10x30x21.tar.gz --directory /jetsonfs

RUN rm -Rf /jetsonfs/opt/ros/melodic/share
RUN tar -xzvf share_patch_11x5x21.tar.gz --directory /jetsonfs/opt/ros/melodic
RUN ln -s /jetsonfs/usr/lib/aarch64-linux-gnu/ /usr/lib/aarch64-linux-gnu

WORKDIR /tmp
RUN rm -Rf ck_jetson_run_tar

RUN apt-get install -y bash-completion
RUN apt-get install -y nano parallel libsdl2-dev

ARG NOW
RUN git clone https://github.com/frcteam195/container_support_files
RUN cat container_support_files/bash.bashrc > /etc/bash.bashrc
RUN rm -Rf /root/.bashrc
RUN rm -Rf /root/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache
RUN ln -s /jetsonfs/lib/aarch64-linux-gnu/ /lib/aarch64-linux-gnu
WORKDIR /mnt/working

RUN rm -Rf /tmp/*


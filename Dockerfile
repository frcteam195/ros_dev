FROM ubuntu:18.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y apt-utils wget software-properties-common x11-apps net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules libboost-all-dev python-pip bash-completion nano parallel libsdl2-dev

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key list
RUN apt-get update
RUN apt-get -y install ros-melodic-desktop-full

RUN echo 'root:robots' | chpasswd

RUN mkdir /mnt/working
WORKDIR /mnt/working

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

RUN mkdir dummy
WORKDIR /tmp/dummy
RUN git init
RUN git lfs install

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/ck_jetson_run_tar.git
WORKDIR /tmp/ck_jetson_run_tar
RUN mkdir -p /jetsonfs
RUN tar -xzvf ck_ros_arm64_12x3x21.tar.gz --directory /jetsonfs

RUN apt-get install -y libsdl-dev libsdl-image1.2-dev libsuitesparse-dev ros-melodic-libg2o

ARG NOW
RUN git pull
RUN rm -Rf /jetsonfs/opt/ros/melodic/share
RUN tar -xzvf share_patch_12x3x21.tar.gz --directory /jetsonfs/opt/ros/melodic
RUN ln -s /jetsonfs/usr/lib/aarch64-linux-gnu/ /usr/lib/aarch64-linux-gnu

WORKDIR /tmp
RUN rm -Rf ck_jetson_run_tar

RUN git clone https://github.com/frcteam195/container_support_files
RUN cat container_support_files/bash.bashrc > /etc/bash.bashrc
RUN rm -Rf /root/.bashrc
RUN rm -Rf /root/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache
RUN ln -s /jetsonfs/lib/aarch64-linux-gnu/ /lib/aarch64-linux-gnu
WORKDIR /mnt/working

RUN rm -Rf /tmp/*


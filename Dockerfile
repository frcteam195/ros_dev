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

RUN printf "\nsource /opt/ros/melodic/setup.bash\n" >> /mnt/.bashrc

RUN apt-get -y install libboost-all-dev
RUN apt-get -y install python-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN pip install -U rosinstall vcstools rospkg

RUN echo "export PS1=\"\e[1;35mck-ros-dev> \[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$\"" >> /root/.bashrc
RUN cp /root/.bashrc /mnt/.bashrc
RUN cp /root/.profile /mnt/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache

RUN printf "umask 002\n" >> /mnt/.bashrc
RUN printf "source /opt/ros/melodic/setup.bash\n" >> /mnt/.bashrc

WORKDIR /tmp
RUN curl -L https://developer.nvidia.com/embedded/dlc/l4t-gcc-7-3-1-toolchain-64-bit -o toolchain.tar.xz
RUN unxz toolchain.tar.xz
RUN tar -xvf toolchain.tar
RUN mv gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu /jetsontoolchain
RUN rm -Rf toolchain.tar

RUN apt-get install git-lfs
RUN git clone https://github.com/frcteam195/ck_jetson_run_tar.git
WORKDIR /tmp/ck_jetson_run_tar
RUN git lfs pull
RUN mkdir -p /jetsonfs
RUN tar -xzvf ck_jetson_run_10x29x21.tar.gz --directory /jetsonfs

WORKDIR /tmp
RUN rm -Rf ck_jetson_run_tar

WORKDIR /mnt/working


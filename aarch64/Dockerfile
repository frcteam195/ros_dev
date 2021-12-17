FROM nvcr.io/nvidia/l4t-base:r32.6.1
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y apt-utils wget software-properties-common x11-apps

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key list
RUN apt-get update
RUN apt-get install -y ros-melodic-desktop-full ros-melodic-libg2o libsdl-dev libsdl-image1.2-dev libsuitesparse-dev net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules python-pip bash-completion nano parallel libsdl2-dev

RUN echo 'root:robots' | chpasswd

RUN mkdir /mnt/working
WORKDIR /mnt/working

RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN pip install -U rosinstall vcstools rospkg

RUN mkdir -p /tmp/dummy
WORKDIR /tmp/dummy
RUN git init
RUN git lfs install

WORKDIR /tmp
RUN git clone https://github.com/stiffstream/sobjectizer
WORKDIR /tmp/sobjectizer
RUN git checkout 972b5310b7a486dd4d4322ffb46f1c7e15c47ef6
RUN mkdir cmake_build
WORKDIR /tmp/sobjectizer/cmake_build
RUN cmake -DCMAKE_INSTALL_PREFIX=target -DCMAKE_BUILD_TYPE=Release ../dev
RUN cmake --build . --config Release
RUN cmake --build . --config Release --target install
WORKDIR /tmp


RUN apt-get install -y ros-melodic-common-msgs ros-melodic-teb-local-planner ros-melodic-robot-localization libfmt-dev libgeographic-dev libgtest-dev

ARG NOW
RUN git clone https://github.com/frcteam195/container_support_files
RUN cat container_support_files/bash.bashrc > /etc/bash.bashrc
RUN rm -Rf /root/.bashrc
RUN rm -Rf /root/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache

RUN rm -Rf /tmp/*

WORKDIR /mnt/working

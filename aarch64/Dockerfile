FROM nvcr.io/nvidia/l4t-base:r32.6.1

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

RUN apt-get -y install python-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN pip install -U rosinstall vcstools rospkg

RUN git clone https://github.com/frcteam195/container_support_files
RUN cat container_support_files/bashrc_additions.txt >> /root/.bashrc
RUN cp /root/.bashrc /mnt/.bashrc
RUN cp /root/.profile /mnt/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache

RUN printf "umask 002\n" >> /mnt/.bashrc
RUN printf "source /opt/ros/melodic/setup.bash\n" >> /mnt/.bashrc
RUN apt-get install -y nano parallel
WORKDIR /mnt/working


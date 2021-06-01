FROM phanect/kubuntu:18.04

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

ENV DONT_PROMPT_WSL_INSTALL=TRUE
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN apt-get update
RUN apt-get install -y code
RUN mkdir /vscodeusr

RUN apt-get install -y vim
RUN apt-get install -y emacs

RUN apt-get install -y git
RUN apt-get install -y extra-cmake-modules
RUN apt-get install -y libkf5*
RUN apt-get remove -y konsole

WORKDIR /root
RUN git clone https://github.com/michaelgtodd/konsole.git
WORKDIR /root/konsole/
RUN mkdir build
WORKDIR /root/konsole/build
RUN cmake ..
RUN make
RUN make install
RUN cp ../data/color-schemes/Otto.colorscheme /usr/share/konsole
RUN cp ../data/color-schemes/Otto.colorscheme /usr/local/share/konsole

WORKDIR /root
RUN rm -rf /root/konsole

RUN mkdir working
WORKDIR /root/working

RUN printf "\nsource /opt/ros/melodic/setup.bash\n" >> /root/.bashrc

RUN apt-get -y install python-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN pip install -U rosinstall vcstools rospkg

RUN printf "umask 000\n" >> /root/.bashrc

RUN printf "#!/bin/bash\nsysctl -w net.ipv4.tcp_keepalive_intvl=60 net.ipv4.tcp_keepalive_probes=5 net.ipv4.tcp_keepalive_time=300" >> /root/.sysctl.sh
RUN printf "/root/.sysctl.sh >/dev/null\n" >> /root/.bashrc
RUN chmod a+x /root/.sysctl.sh

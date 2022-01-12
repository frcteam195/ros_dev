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
RUN apt-get install -y ros-melodic-desktop-full ros-melodic-libg2o libsdl-dev libsdl-image1.2-dev libsuitesparse-dev net-tools iputils-ping vim emacs git git-lfs extra-cmake-modules python-pip bash-completion nano parallel

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
RUN cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ../dev
RUN cmake --build . --config Release
RUN cmake --build . --config Release --target install
WORKDIR /tmp

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSlibzmq.git
WORKDIR /tmp/CKROSlibzmq 
RUN git checkout 4097855ddaaa65ed7b5e8cb86d143842a594eebd
RUN mkdir cppbuild
WORKDIR /tmp/CKROSlibzmq/cppbuild
RUN cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
RUN make -j8
RUN make install

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSprotobuf.git
WORKDIR /tmp/CKROSprotobuf     
RUN git checkout 89b14b1d16eba4d44af43256fc45b24a6a348557
RUN mkdir cppbuild
WORKDIR /tmp/CKROSprotobuf/cppbuild
RUN cmake ../cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
RUN make -j8
RUN make install

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSfmt.git
WORKDIR /tmp/CKROSfmt
RUN git checkout d141cdbeb0fb422a3fb7173b285fd38e0d1772dc
RUN mkdir cppbuild
WORKDIR /tmp/CKROSfmt/cppbuild
RUN cmake ..
RUN make
RUN make install

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKROSSDL.git
WORKDIR /tmp/CKROSSDL
RUN git checkout 2e9821423a237a1206e3c09020778faacfe430be
RUN ./configure
RUN make -j8
RUN make install

RUN apt-get install -y libglfw3 libglfw3-dev

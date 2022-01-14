ARG ARCH
FROM guitar24t/ck-ros-base:${ARCH}
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKimgui.git \
	&& cd /tmp/CKimgui \
	&& make -j4 \
	&& make install \
	&& cd /tmp \
	&& git clone https://github.com/frcteam195/CKimplot.git \
	&& cd /tmp/CKimplot \
	&& make -j4 \
	&& make install \
	&& rm -Rf /tmp/*


RUN apt-get update \
	&& apt-get install -y mesa-utils curl ros-melodic-common-msgs ros-melodic-librealsense2 ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-melodic-teb-local-planner ros-melodic-robot-localization libgeographic-dev libgtest-dev ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3 clang-tidy \
	&& apt-get remove -y ros-melodic-teb-local-planner

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /tmp/catkin_ws/src \
	&& cd /tmp/catkin_ws/src \
	&& git clone https://github.com/rst-tu-dortmund/teb_local_planner.git \
	&& cd /tmp/catkin_ws/src/teb_local_planner \
	&& git checkout ec5759d0bf982cd2266715bc06a3385309ec3eb1 \
	&& cd /tmp/catkin_ws \
	&& source /opt/ros/melodic/setup.bash \
	&& rosdep install --from-paths src --ignore-src -r -y \
	&& catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
	&& catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release \
	&& rm -Rf /tmp/*

#ENV CUDA_HOME=/usr/local/cuda
#ENV CUDA_TOOLKIT_ROOT=/usr/local/cuda
#SHELL ["/bin/bash", "-c"]
#RUN mkdir -p /tmp/catkin_ws/src \
#        && export CUDA_HOME=/usr/local/cuda \
#        && export ENV CUDA_TOOLKIT_ROOT=/usr/local/cuda \
#        && cd /tmp/catkin_ws/src \
#        && git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git \
#        && cd /tmp/catkin_ws/src/zed-ros-wrapper \
#        && git checkout dcd9d972e62c2fd2bd4d3114d70194934fad364c \
#        && cd /tmp/catkin_ws \
#        && source /opt/ros/melodic/setup.bash \
#        && rosdep install --from-paths src --ignore-src -r -y \
#        && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-10.2/targets/aarch64-linux/lib/stubs:/opt/ros/foxy/install/lib \
#        && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DCUDA_HOME=/usr/local/cuda -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs  -DCUDA_CUDART_LIBRARY=/u>
#        && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCUDA_HOME=/usr/local/cuda -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/loca>
#        && rm -Rf /tmp/*

SHELL ["/bin/sh", "-c"]
WORKDIR /tmp
RUN git clone https://github.com/wpilibsuite/allwpilib \
	&& cd /tmp/allwpilib \
	&& git checkout 269cf0347207516016666db24536453387f055cd \
	&& mkdir build \
	&& cd /tmp/allwpilib/build \
	&& cmake -DWITH_JAVA=OFF -DWITH_TESTS=OFF -DWITH_CSCORE=OFF -DWITH_WPIMATH=OFF -DWITH_WPILIB=OFF -DWITH_GUI=OFF -DWITH_SIMULATION_MODULES=OFF -DBUILD_SHARED_LIBS=ON .. \
	&& make -j4 \
	&& make install \
	&& rm -Rf /tmp/*

ARG NOW

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/container_support_files \
	&& cat container_support_files/bash.bashrc > /etc/bash.bashrc \
	&& rm -Rf /root/.bashrc \
	&& rm -Rf /root/.profile \
	&& cp -r /root/.ros /mnt/.ros \
	&& cp -r /root/.cache /mnt/.cache \
	&& rm -Rf /tmp/*

WORKDIR /mnt/working

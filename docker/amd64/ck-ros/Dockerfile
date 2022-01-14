FROM guitar24t/ck-ros-base:amd64
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
	&& apt-get install -y curl ros-melodic-common-msgs ros-melodic-librealsense2 ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-melodic-teb-local-planner ros-melodic-robot-localization libgeographic-dev libgtest-dev ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3 clang-tidy \
	&& apt-get remove -y ros-melodic-teb-local-planner

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /tmp/catkin_ws/src \
	&& cd /tmp/catkin_ws/src \
	&& git clone https://github.com/rst-tu-dortmund/teb_local_planner.git \
	&& cd /tmp/catkin_ws/src/teb_local_planner \
	&& git checkout ec5759d0bf982cd2266715bc06a3385309ec3eb1 \
	&& cd /tmp/catkin_ws \
	&& source /opt/ros/melodic/setup.bash \
	&& catkin_make -j4 \
	&& catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic \
	&& rm -Rf /tmp/*

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

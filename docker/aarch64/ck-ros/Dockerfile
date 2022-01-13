FROM guitar24t/ck-ros-base:arm64
ARG DEBIAN_FRONTEND=noninteractive
ARG NOW

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKimgui.git
WORKDIR /tmp/CKimgui
RUN make -j4
RUN make install

WORKDIR /tmp
RUN git clone https://github.com/frcteam195/CKimplot.git
WORKDIR /tmp/CKimplot
RUN make -j4
RUN make install

RUN apt-get update && apt-get install -y ros-melodic-common-msgs ros-melodic-realsense2-camera ros-melodic-librealsense2 ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-melodic-teb-local-planner ros-melodic-robot-localization libgeographic-dev libgtest-dev ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3 clang-tidy
RUN apt-get remove -y ros-melodic-teb-local-planner
RUN mkdir -p /tmp/catkin_ws/src
WORKDIR /tmp/catkin_ws/src
RUN git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
WORKDIR /tmp/catkin_ws/src/teb_local_planner
RUN git checkout ec5759d0bf982cd2266715bc06a3385309ec3eb1
WORKDIR /tmp/catkin_ws
RUN ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash; catkin_make -j4; catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic"]

WORKDIR /tmp
RUN git clone https://github.com/wpilibsuite/allwpilib
WORKDIR /tmp/allwpilib
RUN git checkout 269cf0347207516016666db24536453387f055cd
RUN mkdir build
WORKDIR /tmp/allwpilib/build
RUN cmake -DWITH_JAVA=OFF -DWITH_TESTS=OFF -DWITH_CSCORE=OFF -DWITH_WPIMATH=OFF -DWITH_WPILIB=OFF -DWITH_GUI=OFF -DWITH_SIMULATION_MODULES=OFF -DBUILD_SHARED_LIBS=ON ..
RUN make -j4 && make install

WORKDIR /tmp

ARG NOW
RUN git clone https://github.com/frcteam195/container_support_files
RUN cat container_support_files/bash.bashrc > /etc/bash.bashrc
RUN rm -Rf /root/.bashrc
RUN rm -Rf /root/.profile
RUN cp -r /root/.ros /mnt/.ros
RUN cp -r /root/.cache /mnt/.cache

RUN rm -Rf /tmp/*

WORKDIR /mnt/working

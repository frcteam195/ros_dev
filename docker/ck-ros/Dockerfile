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

SHELL ["/bin/bash", "-c"]
RUN apt-get update \
	&& apt-get install -y mesa-utils curl ros-melodic-common-msgs ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-melodic-teb-local-planner ros-melodic-robot-localization libgeographic-dev libgtest-dev ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-gazebo ros-melodic-turtlebot3 clang-tidy ros-melodic-ddynamic-reconfigure \
	&& apt-get remove -y ros-melodic-teb-local-planner \
	&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /tmp/catkin_ws/src \
	&& cd /tmp/catkin_ws/src \
	&& git clone https://github.com/rst-tu-dortmund/teb_local_planner.git \
	&& cd /tmp/catkin_ws/src/teb_local_planner \
	&& git checkout ec5759d0bf982cd2266715bc06a3385309ec3eb1 \
	&& cd /tmp/catkin_ws \
	&& source /opt/ros/melodic/setup.bash \
	&& apt-get update \
	&& rosdep install --from-paths src --ignore-src -r -y \
	&& catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
	&& catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release \
	&& rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
ENV CUDA_HOME=/usr/local/cuda
ENV CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
ENV CUDA_NVCC_EXECUTABLE=/usr/local/cuda/bin/nvcc
ENV CUDA_INCLUDE_DIRS=/usr/local/cuda/include
ENV CUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/stubs
ENV CUDA_VERSION=10.2
RUN mkdir -p /tmp/catkin_ws/src \
        && cd /tmp/catkin_ws/src \
        && git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git \
        && cd /tmp/catkin_ws/src/zed-ros-wrapper \
        && git checkout dcd9d972e62c2fd2bd4d3114d70194934fad364c \
        && cd /tmp/catkin_ws \
        && source /opt/ros/melodic/setup.bash \
        && apt-get update \
        && rosdep install --from-paths src --ignore-src -r -y \
	&& sed -i.bkp '/macro(CUDA_FIND_HELPER_FILE _name _extension)/i set(CUDA_CUDART_LIBRARY /usr/local/cuda/lib64/stubs)' /usr/share/cmake-3.10/Modules/FindCUDA.cmake \
        && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DCUDA_CUDART_LIBRARY=${CUDA_CUDART_LIBRARY} -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic \
        && catkin_make -DCMAKE_BUILD_TYPE=Release -DCUDA_CUDART_LIBRARY=${CUDA_CUDART_LIBRARY} -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install \
	&& sed -i.bkp '/set(CUDA_CUDART_LIBRARY \/usr\/local\/cuda\/lib64\/stubs)/d' /usr/share/cmake-3.10/Modules/FindCUDA.cmake \
        && rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
ARG ARCH
RUN mkdir -p /tmp/catkin_ws/src \
        && cd /tmp/catkin_ws/src \
        && git clone https://github.com/IntelRealSense/realsense-ros \
        && cd /tmp/catkin_ws/src/realsense-ros \
        && git checkout f400d682beee6c216052a419f419e95b797255ad \
        && cd /tmp/catkin_ws \
        && source /opt/ros/melodic/setup.bash \
        && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
        && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release \
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

SHELL ["/bin/bash", "-c"]
RUN chmod -R 775 /usr/local/zed
RUN if [[ -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]]; then chmod 775 /etc/udev/rules.d/99-realsense-libusb.rules; fi

SHELL ["/bin/sh", "-c"]
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

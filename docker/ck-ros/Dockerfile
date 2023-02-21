ARG ARCH
FROM guitar24t/ck-ros-base:${ARCH}
ARG DEBIAN_FRONTEND=noninteractive

# SHELL ["/bin/bash", "-c"]
# ARG ARCH
# RUN mkdir -p /tmp/catkin_ws/src \
#         && cd /tmp/catkin_ws/src \
#         && git clone https://github.com/IntelRealSense/realsense-ros \
#         && cd /tmp/catkin_ws/src/realsense-ros \
#         && git checkout f400d682beee6c216052a419f419e95b797255ad \
#         && cd /tmp/catkin_ws \
#         && source /opt/ros/melodic/setup.bash \
#         && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
#         && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release \
#         && rm -Rf /tmp/*

# SHELL ["/bin/bash", "-c"]
# RUN if [[ -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]]; then chmod 775 /etc/udev/rules.d/99-realsense-libusb.rules; fi

SHELL ["/bin/sh", "-c"]
WORKDIR /tmp
RUN git clone https://github.com/nlohmann/json \
	&& cd /tmp/json \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j4 \
	&& make install \
	&& cd /tmp \
	&& rm -Rf /tmp/*

SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && apt-get remove -y ros-noetic-robot-localization \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /tmp/catkin_ws/src \
	&& cd /tmp/catkin_ws/src \
	&& git clone https://github.com/frcteam195/robot_localization.git \
	&& cd /tmp/catkin_ws \
	&& source /opt/ros/noetic/setup.bash \
	&& apt-get update \
	&& rosdep install --from-paths src --ignore-src -r -y \
	&& catkin_make -j4 -DCMAKE_BUILD_TYPE=Release \
	&& catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCMAKE_BUILD_TYPE=Release \
	&& rm -Rf /tmp/* \
	&& rm -rf /var/lib/apt/lists/*




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
	
RUN apt-get update \
	&& apt-get install -y libi2c-dev \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /mnt/working

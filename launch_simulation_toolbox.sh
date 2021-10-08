xhost +

docker run -ti --rm \
	   -e DISPLAY=$DISPLAY \
	   --privileged \
	   -e XAUTHORITY=$XAUTHORITY \
           -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
	   -v $(pwd):/root/working \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   ck_ros \
	   /bin/bash --rcfile /root/.profile
	   

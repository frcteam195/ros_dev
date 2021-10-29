xhost +

export GID=$(id -g)

docker run -ti --rm \
	   -e DISPLAY=$DISPLAY \
	   --privileged \
	   -e XAUTHORITY=$XAUTHORITY \
       -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
	   -v $(pwd):/mnt/working \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   --user $UID:$GID \
	   --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --net=host \
       -e HOME=/mnt/working \
	   --device=/dev/dri:/dev/dri \
	   guitar24t/ck-ros-dev \
	   /bin/bash --rcfile /mnt/.bashrc
	   

xhost +

DISPLAY=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/172.17.0.1\1/'`
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
       --volume="$HOME/.Xauthority:/mnt/working/.Xauthority:rw" \
       --net=host \
       -e HOME=/mnt/working \
	   --device=/dev/dri:/dev/dri \
	   ck_jetson_run \
	   /bin/bash --rcfile /mnt/.bashrc
	   

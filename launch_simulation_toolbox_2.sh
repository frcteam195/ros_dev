xhost +

export GID=$(id -g)
XAUTH=/$HOME/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH
#DISPLAY=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/172.17.0.1\1/'`
docker run -ti --rm \
	   -e DISPLAY=$DISPLAY \
	   --privileged \
	   -e XAUTHORITY=$XAUTH \
       -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
	   -v $(pwd):/mnt/working \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $XAUTH:$XAUTH \
	   --user $UID:$GID \
	   --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --net=host \
       -e HOME=/mnt/working \
	   --device=/dev/dri:/dev/dri \
	   guitar24t/ck-ros-dev \
	   /bin/bash --rcfile /mnt/.bashrc
	   

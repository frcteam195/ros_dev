xhost +

export GID=$(id -g)
XAUTH=/$HOME/.docker.xauth
touch $XAUTH
XDISPL=`xauth nlist $DISPLAY`

if [ ! -z "$XDISPL" ]
then
  echo $XDISPL | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

chmod 777 $XAUTH

if [[ $OSTYPE == 'darwin'* ]]; then
  DISPLAY_CMD=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/host.docker.internal\1/'`
else
  DISPLAY_CMD=$DISPLAY
fi

docker run -ti --rm \
	   -e DISPLAY=$DISPLAY_CMD \
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
	   

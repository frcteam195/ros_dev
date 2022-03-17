export ROS_IP=$(/sbin/ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)
export ROS_MASTER_URI=http://10.1.95.5:11311

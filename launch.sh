#!/bin/bash



#roslaunch 2022_Flatty_Ros_Robot/prod.launch


#temp till I (Mike Todd) figure out roslaunch MGT TBD <-- warning he wont do it
LIB_PATH=/mnt/working/2022_Flatty_Ros_Robot/catkin_ws/devel/lib

nodes_list=(
    "rio_control_node"
    "drivetrain_node"
    "test_data_node"
    "data_streamer_node"
);

kill_nodes(){
    for node_name in "${nodes_list[@]}"
    do
        echo "Killing using rosnode: $node_name"
        rosnode kill /$node_name
    done

    sleep 2

    for node_name in "${nodes_list[@]}"
    do
        echo "Killing pkill: $node_name"
        pkill -9 $node_name
        pkill -9 $node_name
        pkill -9 $node_name
    done

    pkill roscore
}

run_nodes(){
    roscore&

    sleep 3

    for node_name in "${nodes_list[@]}"
    do
        exec_path=$LIB_PATH/$node_name/$node_name
        echo "Running: $exec_path"
        $exec_path &
    done

}

help_text(){
    echo "./launch.sh <run or kill>"
}

if [ $# -eq 0 ]
then
  help_text
fi

case "$1" in
  "run")
    run_nodes
    ;;
  "kill")
    kill_nodes
    ;;
  *)
    help_text
    ;;
esac


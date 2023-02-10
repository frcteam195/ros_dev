#!/bin/bash

SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ROOT=$(cd $SCRIPT_DIR && cd .. && cd *_Robot && pwd)
CATKIN_WS=$(cd $SCRIPT_DIR && cd .. && cd *_Robot && cd catkin_ws && pwd)
OS_ARCHITECTURE=$(arch)
OS_NAME=$(uname -a)
source "${SCRIPT_DIR}/useful_scripts.sh"

help_text ()
{
		errmsg "No arguments provided, supported arguments are:\n\tbuild \n\tcheckout \n\tclone \n\tclean \n\tcleanlibs \n\tcleanros \n\tcommit \n\tconfigurator \n\tdeletetag \n\tdeploy \n\tnode \n\tnode_python \n\tpush \n\trebuild \n\trebuildlibs \n\trebuildros \n\treclone \n\tshort \n\tstatus \n\ttag \n\ttest \n\tupdate"
}

node_help_text()
{
		errmsg "\nUsage: mkrobot.sh node my_new_node https://new.gitrepo.link\n"
}

rebuild()
{
		clean
		build
}

rebuildros()
{
		cleanros
		build
}

rebuildlibs()
{
		cleanlibs
		build
}

update_prev()
{
		source $SCRIPT_DIR/useful_scripts.sh
		cd $SCRIPT_DIR/..
		forall git pull
		cd $SCRIPT_DIR/../third_party_libs
		forall git pull
}

update()
{
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} pull"
}

status()
{
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} status"
}

short()
{
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} short"
}

reclone()
{
	cd $SCRIPT_DIR/..
	rm -Rf ./*trajectories*
	rm -Rf ./*_node
	update
	clone
}

checkout()
{
	if [ $# -eq 0 ]
	then
		errmsg "\nTag name is not specified. Please enter a tag name"
		return 1;
	fi
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} checkout ${1}"
}

configurator()
{
	source $SCRIPT_DIR/useful_scripts.sh
	cd $SCRIPT_DIR/..
	/usr/bin/env python3 ./ros_dev/configurator.py
}

commit()
{
	if [ $# -eq 0 ]
	then
		errmsg "\nCommit message is not specified. Please enter a commit message"
		return 1;
	fi
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} add -A; git -C {} commit -m ${1}"
}

push()
{
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} push; git -C {} push origin --tags"
}

tag()
{
	if [ $# -lt 1 ]
	then
		errmsg "\Tag version is not specified. Please enter a tag version"
		return 1;
	fi
	if [ $# -lt 2 ]
	then
		errmsg "\Tag message is not specified. Please enter a tag message"
		return 1;
	fi
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} tag -a ${1} -m \"${2}\""
}

deletetag()
{
	if [ $# -lt 1 ]
	then
		errmsg "\Tag version is not specified. Please enter a tag version"
		return 1;
	fi
		if ! command -v parallel &> /dev/null
		then
				infomsg "Installing parallel..."
				sudo apt-get update
				sudo apt-get install -y parallel
		fi
		find . -name ".git" -type d -exec dirname {} \; | parallel -k "echo {}; git -C {} tag -d ${1}; git -C {} push origin :refs/tags/${1}"
}

source_setup_bash()
{
	if [ -f "$CATKIN_WS/devel/setup.bash" ]
	then
		source ~/*_Robot/catkin_ws/devel/setup.bash
		echo "Sourcing setup.bash"
	else
		echo "Can't source setup.bash"
		echo "Is robot built properly?"
	fi
}

launch()
{
	source_setup_bash
	if [ $# -eq 0 ]
	then
		LAUNCH_FILE="${ROBOT_ROOT}/launch/local.launch"
	else
		if [[ ${1} == *.launch ]]
		then
			LAUNCH_FILE="${1}"
		else
			LAUNCH_FILE="${1}.launch"
		fi

		if [[ ${1} != */* ]]
		then
			LAUNCH_FILE="${ROBOT_ROOT}/launch/${LAUNCH_FILE}"
		fi
	fi
	echo "Using launchfile ${LAUNCH_FILE}"

	echo "Deploying Trajectories..."
	mkdir -p ./tmptraj
	rm -Rf ./tmptraj/**
	cp ${TRAJ_DIR}/**/*.json ./tmptraj/
	cp ${TRAJ_DIR}/*.json ./tmptraj/ 2>>/dev/null

	roslaunch "${LAUNCH_FILE}"
}

deploy()
{
	if [ ! -f /.dockerenv ]; then
		infomsg "This command must be run in a docker container. Running in docker for you..."

		cd $SCRIPT_DIR/..
		./ros_dev/run_container.sh -f -c "/mnt/working/ros_dev/mkrobot.sh deploy"
		return;
        fi
        exit_if_not_docker

	TARGET_IP=${1:-10.1.95.5}

	BASEDIR=$(dirname "$0")
	ROOT_DIR=$(realpath $(dirname ${BASEDIR}))
	source "${BASEDIR}/useful_scripts.sh"
	OS_ARCHITECTURE=$(arch)

	if [[ $(pwd) == *"ros_dev"* ]]; then
	    infomsg "This script cannot be run from this directory. Attempting to fix..."
	    cd ..
	    if [ -d "$(pwd)/$(ls | grep *_Robot)" ]; then
	        infomsg "Correct directory found. Launching..."
	    else
	        errmsg "Unable to detect the proper directory to run from..."
	        exit 1
	    fi
	fi

	#ROSLIB_PATH="outputs/aarch64/devel/lib"
	#if [ $OS_ARCHITECTURE == 'aarch64' ] || [ $OS_ARCHITECTURE == 'arm64' ]
	#then
	#    ROSLIB_PATH="outputs/native/devel/lib"
	#fi

	BASE_PATH=$(find . -maxdepth 1 -type d -name '*_Robot*' -print -quit | xargs realpath -P)
	#FULL_ROSLIB_PATH="${BASE_PATH}/${ROSLIB_PATH}"
	#cd ${FULL_ROSLIB_PATH}
	cd ${BASE_PATH}/..

	cd ./*trajectories_*
	if [ $? -eq 0 ]; then
		TRAJ_DIR=$(pwd)
		cd ..
		echo "Deploying Trajectories..."
		mkdir -p ./tmptraj
		rm -Rf ./tmptraj/**
		cp ${TRAJ_DIR}/**/*.json ./tmptraj/
		cp ${TRAJ_DIR}/*.json ./tmptraj/ 2>>/dev/null
		ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no team195@${TARGET_IP} 'rm -Rf /robot/trajectories/* && mkdir -p /robot/trajectories && chown team195:team195 /robot/trajectories'
		scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no ./tmptraj/*.json team195@${TARGET_IP}:/robot/trajectories
	else
		echo "No trajectories found"
	fi

	echo "Cleaning PyCache..."
	rm -Rf *_Robot/**/__pycache__
	echo "Packing robot..."
	tar -hczf ${ROOT_DIR}/rosdeploy.tar.gz *_Robot/*
	cd  ${ROOT_DIR}
	echo "Deploying robot to target..."
	scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no rosdeploy.tar.gz  team195@${TARGET_IP}:/robot
	echo "Unpacking robot on target..."
	ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no team195@${TARGET_IP} '/robot/ros_scripts/unpackros.sh'
	echo "Done!"
}

node()
{
	if [ -z "${1}" ]; then
		errmsg "\nNode name is not specified. Please enter a node name." "noexit"
		node_help_text
		return
	fi
	if [[ "${1}" = *[[:space:]]* ]]
	then
		errmsg "\nPlease enter a node name that does not have any spaces."
		node_help_text
		return
	fi

	if [[ "${1}" != *_node ]]
	then
		errmsg "\nPlease enter a node name that ends in node."
		node_help_text
		return
	fi

	cd $SCRIPT_DIR/..
	git clone git@github.com:frcteam195/template_node.git
	rm -Rf template_node/.git

	mv template_node/ "${1}/"
	cd ${1}
	find . -type f | grep -v ^.$ | xargs sed -i "s/tt_node/${1}/g"
	find . -type f | grep -v ^.$ | xargs sed -i "s/template_node/${1}/g"
	mv src/tt_node.cpp "src/${1}.cpp"
	mv include/tt_node.hpp "include/${1}.hpp"
	mv test/src/test_tt_node.cpp "test/src/test_${1}.cpp"
	mv test/include/test_tt_node.hpp "test/include/test_${1}.hpp"

	if [ -z "${2}" ]; then
		return
	fi
	cd $SCRIPT_DIR/..
	git clone "${2}" temp_repo
	shopt -s dotglob
	mv temp_repo/* "${1}"
	cd "${1}"
	git add -A
	git commit -m "Initial commit"
	git push
	cd $SCRIPT_DIR/..
	rm -Rf temp_repo/
}

node_python()
{
	if [ -z "${1}" ]; then
		errmsg "\nNode name is not specified. Please enter a node name." "noexit"
		node_help_text
		return
	fi
	if [[ "${1}" = *[[:space:]]* ]]
	then
		errmsg "\nPlease enter a node name that does not have any spaces."
		node_help_text
		return
	fi

	if [[ "${1}" != *_node ]]
	then
		errmsg "\nPlease enter a node name that ends in node."
		node_help_text
		return
	fi

	node_name=${1}
	class_name=$(echo "$node_name" | sed -e "s/_/ /g" | sed -e "s/\b\(.\)/\u\1/g" | sed "s/[[:blank:]]//g")

	cd $SCRIPT_DIR/..
	git clone git@github.com:frcteam195/template_python_node.git
	rm -Rf template_python_node/.git

	mv template_python_node/ "$node_name/"
	cd $node_name
	find . -type f | grep -v ^.$ | xargs sed -i "s/template_python_node/$node_name/g"
	find . -type f | grep -v ^.$ | xargs sed -i "s/TemplatePythonNode/$class_name/g"
	mv scripts/template_python_node scripts/$node_name
	mv src/template_python_node src/$node_name

	if [ -z "${2}" ]; then
		return
	fi
	cd $SCRIPT_DIR/..
	git clone "${2}" temp_repo
	shopt -s dotglob
	mv temp_repo/* "$node_name"
	cd "$node_name"
	git add -A
	git commit -m "Initial commit"
	git push
	cd $SCRIPT_DIR/..
	rm -Rf temp_repo/
}

cleanlibs ()
{
	cd $SCRIPT_DIR/..

	find . -maxdepth 1 2>/dev/null | grep -v ^.$ | grep -v "^\./\." | grep -v  ".*_node" | grep -v  ".*_Robot" | grep -v third_party_libs | grep -v ros_dev | xargs -I {} sh -c "echo 'Attempting to clean {}' && cd {} && make clean"

	if [ -d "./third_party_libs" ]
	then
		echo Cleaning third party libraries...
		cd third_party_libs
		find . -maxdepth  1 | grep -v ^.$ | xargs -I {} sh -c "echo 'Attempting to clean {}' && cd {} && make clean"
	fi
}

cleanros ()
{
	cd $SCRIPT_DIR/..

	cd *_Robot/
	mkdir -p catkin_ws/src
	cd catkin_ws/src
	find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
	find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -v ".*_Robot" | grep -v ^../../../third_party_libs$$
	cd ..
	catkin_make clean

	rm -rf /mnt/working/*_Robot/outputs/*/build/*
	rm -rf /mnt/working/*_Robot/outputs/*/devel/*
}

clean ()
{
	cleanlibs
	cleanros
}

clone ()
{
	cd $SCRIPT_DIR/..
	cat *_Robot/ros_projects.txt | xargs -I {} git clone {}
	mkdir -p third_party_libs
	cd third_party_libs
	cat ../*_Robot/third_party_projects.txt | sed -r -e "/^#.*$/d" | xargs -I {} git clone {}
}

build ()
{
	if [ $OS_ARCHITECTURE == 'arm64' ]
	then
		OS_ARCHITECTURE="aarch64"
	fi

	if [ ! $OS_ARCHITECTURE == 'aarch64' ] && [ ! -f /.dockerenv ]
	then
		docker run --rm --privileged multiarch/qemu-user-static --reset -p yes > /dev/null
	fi

	if [ $# -eq 0 ]
	then
		BUILD_ARCHITECTURE="${OS_ARCHITECTURE}"
	else
		BUILD_ARCHITECTURE=$1
	fi

	if [ $OS_ARCHITECTURE != $BUILD_ARCHITECTURE ] && [ -f /.dockerenv ]
	then
		errmsg "A cross compile must be run directly from the host and not inside a container."
		exit
	fi

	case "$BUILD_ARCHITECTURE" in
		"x86_64")
			;;
		"aarch64")
			;;
		*)
			errmsg "Invalid architecture \"$1\" supported architectures are: x86_64 aarch64"
			exit
			;;
	esac

	infomsg "Targeting $BUILD_ARCHITECTURE"

	cd $SCRIPT_DIR/..
	find -name "._*" -delete

	cd *_Robot
	mkdir -p catkin_ws
	cd ..

	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/build
	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/devel

	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$BUILD_ARCHITECTURE/build {}/catkin_ws/build
	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$BUILD_ARCHITECTURE/devel {}/catkin_ws/devel


	DOCKER_FLAGS=
	case "${BUILD_ARCHITECTURE}" in
		"x86_64")
			DOCKER_FLAGS="-i"
			;;
		"aarch64")
			DOCKER_FLAGS="-a"
			;;
		*)
			;;
	esac

	if [ ${OS_ARCHITECTURE} = ${BUILD_ARCHITECTURE} ]
	then
		if [ ! -f /.dockerenv ]; then
			infomsg "This command must be run in a docker container. Running in docker for you..."

			cd $SCRIPT_DIR/..
			./ros_dev/run_container.sh ${DOCKER_FLAGS} -f -c "/mnt/working/ros_dev/mkrobot.sh build ${BUILD_ARCHITECTURE}"
			return;
		fi
		exit_if_not_docker
		if [ -d "./third_party_libs" ]
		then
			infomsg "Making third party libraries..."
			cd third_party_libs
			cat ../*_Robot/third_party_projects.txt | grep -v "^#.*$" | sed s:^.*/::g | sed s:.git.*$::g | xargs -I {} sh -c "echo 'Attempting to make {}' && cd {} && if [ -f \"Makefile\" ]; then make ${BUILD_ARCHITECTURE}; fi"
		fi

		cd $SCRIPT_DIR/..

		cd *_Robot
		mkdir -p catkin_ws/src
		cd catkin_ws/src
		find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
		find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -e ".*_node" -e ".*_planner" | sed s:../../../::g | xargs -I {} ln -s ../../../{} {}
		cd ..
		catkin_make -j$(($(nproc)-1)) -DROBOT_ARCHITECTURE_${OS_ARCHITECTURE^^}=TRUE -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" -DCATKIN_ENABLE_TESTING=0
	elif [ ${OS_ARCHITECTURE} != ${BUILD_ARCHITECTURE} ]
	then
		./ros_dev/run_container.sh ${DOCKER_FLAGS} -f -c "/mnt/working/ros_dev/mkrobot.sh build ${BUILD_ARCHITECTURE}"
	else
		errmsg "Build case not identified!"
	fi

}

builddebug ()
{
	if [ $OS_ARCHITECTURE == 'arm64' ]
	then
		OS_ARCHITECTURE="aarch64"
	fi

	if [ ! $OS_ARCHITECTURE == 'aarch64' ] && [ ! -f /.dockerenv ]
	then
		docker run --rm --privileged multiarch/qemu-user-static --reset -p yes > /dev/null
	fi

	if [ $# -eq 0 ]
	then
		BUILD_ARCHITECTURE="${OS_ARCHITECTURE}"
	else
		BUILD_ARCHITECTURE=$1
	fi

	if [ $OS_ARCHITECTURE != $BUILD_ARCHITECTURE ] && [ -f /.dockerenv ]
	then
		errmsg "A cross compile must be run directly from the host and not inside a container."
		exit
	fi

	case "$BUILD_ARCHITECTURE" in
		"x86_64")
			;;
		"aarch64")
			;;
		*)
			errmsg "Invalid architecture \"$1\" supported architectures are: x86_64 aarch64"
			exit
			;;
	esac

	infomsg "Targeting $BUILD_ARCHITECTURE"

	cd $SCRIPT_DIR/..
	find -name "._*" -delete

	cd *_Robot
	mkdir -p catkin_ws
	cd ..

	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/build
	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/devel

	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$BUILD_ARCHITECTURE/build {}/catkin_ws/build
	find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$BUILD_ARCHITECTURE/devel {}/catkin_ws/devel


	DOCKER_FLAGS=
	case "${BUILD_ARCHITECTURE}" in
		"x86_64")
			DOCKER_FLAGS="-i"
			;;
		"aarch64")
			DOCKER_FLAGS="-a"
			;;
		*)
			;;
	esac

	if [ ${OS_ARCHITECTURE} = ${BUILD_ARCHITECTURE} ]
	then
		if [ ! -f /.dockerenv ]; then
			infomsg "This command must be run in a docker container. Running in docker for you..."

			cd $SCRIPT_DIR/..
			./ros_dev/run_container.sh ${DOCKER_FLAGS} -f -c "/mnt/working/ros_dev/mkrobot.sh build ${BUILD_ARCHITECTURE}"
			return;
		fi
		exit_if_not_docker
		if [ -d "./third_party_libs" ]
		then
			infomsg "Making third party libraries..."
			cd third_party_libs
			cat ../*_Robot/third_party_projects.txt | grep -v "^#.*$" | sed s:^.*/::g | sed s:.git.*$::g | xargs -I {} sh -c "echo 'Attempting to make {}' && cd {} && if [ -f \"Makefile\" ]; then make ${BUILD_ARCHITECTURE}; fi"
		fi

		cd $SCRIPT_DIR/..

		cd *_Robot
		mkdir -p catkin_ws/src
		cd catkin_ws/src
		find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
		find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -e ".*_node" -e ".*_planner" | sed s:../../../::g | xargs -I {} ln -s ../../../{} {}
		cd ..
		catkin_make -j$(($(nproc)-1)) -DROBOT_ARCHITECTURE_${OS_ARCHITECTURE^^}=TRUE -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" -DCATKIN_ENABLE_TESTING=0
	elif [ ${OS_ARCHITECTURE} != ${BUILD_ARCHITECTURE} ]
	then
		./ros_dev/run_container.sh ${DOCKER_FLAGS} -f -c "/mnt/working/ros_dev/mkrobot.sh build ${BUILD_ARCHITECTURE}"
	else
		errmsg "Build case not identified!"
	fi

}

mkrobot_test ()
{
	exit_if_not_docker

	shift

	if [ $# -eq 0 ]
	then
		errmsg "You must specify at least one node to test:\n\tmkrobot.sh test rio_control_node legacy_logstreamer_node"
	fi

	cd ${CATKIN_WS}
	catkin_make -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" -DCATKIN_ENABLE_TESTING=1
	BASE_COMMAND="catkin_make"
	BASE_TEST_ARG="run_tests_"
	FULL_ARGS="${BASE_COMMAND}"
	for node in "$@"
	do
		FULL_ARGS="${FULL_ARGS} ${BASE_TEST_ARG}${node}"
	done
	roscore &
	ROSCORE_ID=$!
	sleep 2
	echo "Running command: ${FULL_ARGS}"
	${FULL_ARGS}
	pkill roscore
	wait ${ROSCORE_ID}
}


if [ $# -eq 0 ]
then
	help_text
fi

case "$1" in
	"build")
		build $2
		;;
	"builddebug")
		builddebug
		;;
	"checkout")
		checkout "${2}"
		;;
	"clone")
		clone
		;;
	"clean")
		clean
		;;
	"cleanlibs")
		cleanlibs
		;;
	"cleanros")
		cleanros
		;;
	"commit")
		commit "$@"
		;;
	"configurator")
		configurator
		;;
	"deletetag")
		deletetag "${2}"
		;;
	"deploy")
		deploy "${2}"
		;;
	"node")
		node "${2}" "${3}"
		;;
	"node_python")
		node_python "${2}" "${3}"
		;;
	"push")
		push
		;;
	"rebuild")
		rebuild
		;;
	"rebuildlibs")
		rebuildlibs
		;;
	"rebuildros")
		rebuildros
		;;
	"reclone")
		reclone
		;;
	"short")
		short
		;;
	"status")
		status
		;;
	"tag")
		tag "${2}" "${3}"
		;;
	"test")
		mkrobot_test "$@"
		;;
	"update")
		update
		;;
	"launch")
		launch $2
		;;
	*)
		help_text
		;;
esac


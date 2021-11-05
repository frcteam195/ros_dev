#!/bin/bash

SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

help_text () 
{
  echo -e "No arguments provided, supported  are:\n\tbuild \n\tclone \n\tclean \n\trebuild \n\tupdate"
  exit 1
}

rebuild()
{
    clean
    build
}

update()
{
    source $SCRIPT_DIR/useful_scripts.sh
    cd $SCRIPT_DIR/..
    forall git pull
    cd $SCRIPT_DIR/../third_party_libs
    forall git pull
}

clean ()
{
  cd $SCRIPT_DIR/..

  if [ -d "./third_party_libs" ]
  then
    echo Cleaning third party libraries...
    cd third_party_libs
    find . -maxdepth  1 | grep -v ^.$ | xargs -I {} sh -c "echo 'Attempting to clean {}' && cd {} && make clean"
  fi

  cd $SCRIPT_DIR/..

  cd *_Robot/catkin_ws/src
  find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
  find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -v ".*_Robot" | grep -v ^../../../third_party_libs$ | sed s:../../../::g | xargs -I {} ln -s ../../../{} {}
  cd .. 
  catkin_make clean

  rm -rf /mnt/working/*_Robot/outputs/*/build/*
  rm -rf /mnt/working/*_Robot/outputs/*/devel/*
}

clone ()
{
  cd $SCRIPT_DIR/..
  cat *_Robot/ros_projects.txt | xargs -I {} git clone {}
  mkdir -p third_party_libs
  cd third_party_libs
  cat ../*_Robot/third_party_projects.txt | xargs -I {} git clone {}
}

build ()
{
    
  if [ $# -eq 0 ]
  then
    ARCHITECTURE="x86_64"
  else
    ARCHITECTURE=$1
  fi

  case "$ARCHITECTURE" in
    "x86_64")
      ;;
    "aarch64")
      ;;
    *)
      echo "Invalid architecture \"$1\" supported architectures are: x86_64 aarch64"
      exit
      ;;
  esac

  echo "Targetting $ARCHITECTURE"

  cd $SCRIPT_DIR/..

  find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/build
  find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} rm -rf {}/catkin_ws/devel

  find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$ARCHITECTURE/build {}/catkin_ws/build
  find . | grep _Robot$ | xargs -I {} realpath {} | xargs -I {} ln -s {}/outputs/$ARCHITECTURE/devel {}/catkin_ws/devel

  case "$ARCHITECTURE" in
    "x86_64")
        if [ -d "./third_party_libs" ]
        then
          echo Making third party libraries...
          cd third_party_libs
          find . -maxdepth  1 | grep -v ^.$ | xargs -I {} sh -c "echo 'Attempting to make {}' && cd {} && make x86_64"
        fi
      ;;
    "aarch64")
        if [ -d "./third_party_libs" ]
        then
          echo Making third party libraries...
          cd third_party_libs
          find . -maxdepth  1 | grep -v ^.$ | xargs -I {} sh -c "echo 'Attempting to make {}' && cd {} && make aarch64"
        fi
      ;;
    *)
      ;;
  esac
  


  cd $SCRIPT_DIR/..

  cd *_Robot/catkin_ws/src
  find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
  find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -v ".*_Robot" | grep -v ^../../../third_party_libs$ | sed s:../../../::g | xargs -I {} ln -s ../../../{} {}
  cd ..

  case "$ARCHITECTURE" in
    "x86_64")
      catkin_make -DROBOT_ARCHITECTURE_X86_64=TRUE
      ;;
    "aarch64")
      catkin_make \
        -DROBOT_ARCHITECTURE_AARCH64=TRUE \
        -DCMAKE_TOOLCHAIN_FILE=/mnt/working/ros_dev/l4t_dockerfile/aarch64_jetson_toolchain.cmake \
        -DJETSON_TOOLCHAIN_PATH=/jetsontoolchain/bin/ \
        -DCATKIN_ENABLE_TESTING=OFF
      ;;
    *)
      echo "Invalid architecture \"$1\" supported architectures are: x86_64 aarch64"
      exit
      ;;
  esac

  
}

if [ $# -eq 0 ]
then
  help_text
fi

case "$1" in
  "build")
    build $2
    ;;
  "clone")
    clone
    ;;
  "clean")
    clean
    ;;
  "rebuild")
    rebuild
    ;;
  "update")
    update
    ;;
  *)
    help_text
    ;;
esac


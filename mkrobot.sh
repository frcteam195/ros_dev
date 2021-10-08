#!/bin/bash

SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Makin that robot"

cd $SCRIPT_DIR/..

if [ -d "./third_party_libs" ]
then
  echo Making third party libraries...
  cd third_party_libs
  find . -maxdepth  1 | grep -v ^.$ | xargs -I {} sh -c "echo 'Attempting to make {}' && cd {} && make"
fi

cd $SCRIPT_DIR/..

cd *_Robot/catkin_ws/src
find . -maxdepth 1 | grep -v ^.$ | grep -v ^./CMakeLists.txt$ | xargs -I {} rm {}
find ../../.. -maxdepth 1 2>/dev/null | grep -v ^../../..$ | grep -v ".*_Robot" | grep -v ^../../../third_party_libs$ | sed s:../../../::g | xargs -I {} ln -s ../../../{} {}
cd ..
catkin_make


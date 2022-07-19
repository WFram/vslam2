#!/bin/bash
trap : SIGTERM SIGINT

function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 LAUNCH_FILE" >&2
  exit 1
fi

roscore &
ROSCORE_PID=$!
sleep 1

RVIZ_PID=$!

docker run \
  -it \
  --rm \
  --net=host \
  ros:vslam \
  /bin/bash -c \
  "cd /root/catkin_ws/; \
     source devel/setup.bash; \
     roslaunch vslam2 ${1}"

wait $ROSCORE_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
fi
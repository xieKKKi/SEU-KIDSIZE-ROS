#!/bin/bash
export LD_LIBRARY_PATH=/usr/local/webots/lib:$LD_LIBRARY_PATH
rosrun simulation simrobot &
exit 0
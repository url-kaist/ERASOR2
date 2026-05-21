#!/bin/bash
set +e
SEQ=$1
BAG=/data/bags/${SEQ}_node.bag

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

# Start roscore separately first
roscore > /tmp/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 5
echo "roscore started PID=$ROSCORE_PID"

# Verify roscore is up
rosnode list 2>&1 || { echo "roscore failed to start"; cat /tmp/roscore.log; exit 1; }

# Set rosparams manually (avoid rosparam subst issues in launch)
rosparam set /map/target_rosbag "$(basename $BAG)"
rosparam set /map/save_path "/data/outputs"
rosparam set /map/voxelsize 0.2
rosparam set /map/viz_interval 100
rosparam set /large_scale/is_large_scale true
rosparam set /large_scale/submap_size 200.0

# Start mapgen
rosrun erasor kitti_mapgen > /tmp/mapgen.log 2>&1 &
MAPGEN_PID=$!
sleep 3
echo "mapgen started PID=$MAPGEN_PID"
rosnode list

# Play bag
echo "playing bag..."
rosbag play -r 1 --quiet "$BAG"
sleep 20

echo "--- mapgen.log tail ---"
tail -30 /tmp/mapgen.log
echo "--- outputs ---"
ls -la /data/outputs/

# Cleanup
kill $MAPGEN_PID $ROSCORE_PID 2>/dev/null || true
sleep 2

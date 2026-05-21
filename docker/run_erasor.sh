#!/bin/bash
set +e
SEQ=$1
BAG_NAME=$2

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

roscore > /tmp/roscore.log 2>&1 &
sleep 4
rosparam load /catkin_ws/src/erasor/config/seq_${SEQ}.yaml
rosparam set /large_scale/is_large_scale true
rosparam set /large_scale/submap_size 200.0

rosrun erasor offline_map_updater > /tmp/erasor.log 2>&1 &
ERASOR_PID=$!
sleep 6

rosbag play -r 1 --quiet /data/bags/${BAG_NAME}_node.bag
sleep 30  # let last callbacks drain

# Publish save flag in a way that latches.
echo "publishing save flag..."
rostopic pub -l /saveflag std_msgs/Float32 "data: 0.2" &
PUB_PID=$!
# Poll for the result file to appear, max 60s.
for i in $(seq 1 60); do
  if [ -f /data/outputs/${SEQ}_result.pcd ]; then
    echo "result file appeared after $i seconds"; break
  fi
  sleep 1
done
kill $PUB_PID 2>/dev/null || true

echo "--- last erasor.log (50 lines) ---"
tail -50 /tmp/erasor.log
echo "--- outputs ---"
ls -la /data/outputs/

kill $ERASOR_PID 2>/dev/null || true
pkill -f rosmaster 2>/dev/null || true
sleep 2

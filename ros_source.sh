# ROS2
# check if the paths are not in $COLCON_PREFIX_PATH

# ROS2
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
# ur workspace
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/ur_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/ur_ws/install/setup.bash
  echo "ur_ws sourced"
fi
# controllers
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/controller_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/controller_ws/install/setup.bash
  echo "controller_ws sourced"
fi
# force torque sensor
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/bota_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/bota_ws/install/setup.bash
  echo "force_torque_sensor_ws sourced"
fi
# ultrasound workspace
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/ultrasound_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/ultrasound_ws/install/setup.bash
  echo "ultrasound_ws sourced"
fi
# haptic interface
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nardi/Ultrasound-Demo/haption_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/haption_ws/install:"* ]]; then
    source /home/nardi/Ultrasound-Demo/haption_ws/install/setup.bash
    echo "haption_ws sourced"
fi
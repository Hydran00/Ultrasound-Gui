# ROS2
# check if the paths are not in $COLCON_PREFIX_PATH

# ROS2
if [[ ":$COLCON_PREFIX_PATH:" != *":/opt/ros/humble:"* ]]; then
  source /opt/ros/humble/setup.bash
fi
# ur workspace
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/ur_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/ur_ws/install/setup.bash
fi
# controllers
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/controller_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/controller_ws/install/setup.bash
fi
# force torque sensor
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/robot_ws/force_torque_sensor_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/robot_ws/bota_ws/install/setup.bash
fi
# ultrasound workspace
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/ultrasound_ws/install:"* ]]; then
  source /home/nardi/Ultrasound-Demo/ultrasound_ws/install/setup.bash
fi
# haptic interface
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/hydran00/haption_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
if [[ ":$COLCON_PREFIX_PATH:" != *":/home/nardi/Ultrasound-Demo/haption_ws/install:"* ]]; then
    source /home/nardi/Ultrasound-Demo/haption_ws/install/setup.bash
fi


# ROS2
# check if the paths are not in $COLCON_PREFIX_PATH

# ROS2
export ROS_LOCALHOST_ONLY=1
source /opt/ros/$ROS_DISTRO/setup.zsh

# extract the path to the Ultrasound-Demo folder
# this sould return something like "/home/username/Ultrasound-Demo"
base_path="${PWD%%/Ultrasound-Demo/*}/Ultrasound-Demo"


# ur workspace
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/follower/robot_setup/ur_ws/install:"* ]]; then
  source ${base_path}/follower/robot_setup/ur_ws/install/local_setup.zsh
  echo "ur_ws sourced"
fi

# controllers
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/follower/robot_setup/controller_ws/install:"* ]]; then
  source ${base_path}/follower/robot_setup/controller_ws/install/local_setup.zsh
  echo "controller_ws sourced"
fi

# force torque sensor
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/follower/robot_setup/bota_ws/install:"* ]]; then
  source ${base_path}/follower/robot_setup/bota_ws/install/local_setup.zsh
  echo "force_torque_sensor_ws sourced"
fi

# ultrasound workspace (leader)
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/leader/ros_ws_leader/install:"* ]]; then
  source ${base_path}/leader/ros_ws_leader/install/local_setup.zsh
  echo "leader ros workspace sourced"
fi

# ultrasound workspace (follower)
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/follower/ros_ws_follower/install:"* ]]; then
  source ${base_path}/follower/ros_ws_follower/install/local_setup.zsh
  echo "follower ros workspace sourced"
fi

# haptic interface
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${base_path}/leader/haption_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
if [[ ":$COLCON_PREFIX_PATH:" != *":${base_path}/leader/haption_ws/install:"* ]]; then
    source ${base_path}/leader/haption_ws/install/local_setup.zsh
    echo "haption_ws sourced"
fi
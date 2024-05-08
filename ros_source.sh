# ROS2
. /opt/ros/humble/setup.bash
# driver
. /home/nardi/ur3e-setup/ur_ws/install/setup.bash
# controller
. /home/nardi/ur3e-setup/controller_ws/install/setup.bash
# force/torque sensor
. /home/nardi/ur3e-setup/bota_ws/install/setup.bash
# controllers
. /home/nardi/ultrasound_ws/install/setup.bash
# haptic interface
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nardo/haption_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
source /home/nardi/haption_ws/install/setup.bash



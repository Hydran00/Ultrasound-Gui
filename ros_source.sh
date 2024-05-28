# ROS2
. /opt/ros/humble/setup.bash
# driver
. /home/hydran00/Ultrasound-Env/ur3e-setup/ur_ws/install/setup.bash
# controller
. /home/hydran00/Ultrasound-Env/ur3e-setup/controller_ws/install/setup.bash
# force/torque sensor
. /home/hydran00/Ultrasound-Env/ur3e-setup/bota_ws/install/setup.bash
# controllers
. /home/hydran00/Ultrasound-Env/ultrasound_ws/install/setup.bash
# haptic interface
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/hydran00/haption_ws/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-2.35/
source /home/hydran00/Ultrasound-Env/haption_ws/install/setup.bash



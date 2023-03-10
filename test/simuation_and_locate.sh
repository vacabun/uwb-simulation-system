
CURRENT_DIR=$(cd $(dirname $0); pwd)

WS_DIR=$(cd $(dirname $0);cd ..; pwd)

source /opt/ros/humble/setup.sh
source $WS_DIR/install/setup.sh

ros2 launch $WS_DIR/launch/uwb_simulation_and_locate.launch



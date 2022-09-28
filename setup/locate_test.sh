source /opt/ros/foxy/setup.sh

CURRENT_DIR=$(cd $(dirname $0); pwd)

WS_DIR=$(cd $(dirname $0);cd ..; pwd)


. $WS_DIR/install/setup.bash

ros2 run uwb_location uwb_locate


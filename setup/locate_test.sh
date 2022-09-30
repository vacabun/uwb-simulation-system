
CURRENT_DIR=$(cd $(dirname $0); pwd)

WS_DIR=$(cd $(dirname $0);cd ..; pwd)


. $WS_DIR/install/setup.sh

ros2 run uwb_location uwb_locate


source ../../devel/setup.sh

export ROS_IP="$(echo -e "$(hostname -I)" | sed -e 's/[[:space:]]*$//')"
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI="http://$ROS_IP:11311"
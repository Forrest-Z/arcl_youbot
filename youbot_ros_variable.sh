#!/usr/bin/env sh

echo 'export ROS_IP="$(echo -e "$(hostname -I)" | sed -e 's/[[:space:]]*$//')"' >> ~/.bashrc
echo 'export ROS_HOSTNAME=$ROS_IP' >> ~/.bashrc
echo 'export ROS_MASTER_URI="http://"$(echo "${SSH_CLIENT%% *}")":11311"' >> ~/.bashrc
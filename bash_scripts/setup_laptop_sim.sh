#!/bin/bash

# ----- Delete ROS_MASTER_URI -----

# Delete ROS_MASTER_URI from ~/.bashrc
if grep -q "export ROS_MASTER_URI=" ~/.bashrc; then
    # If ROS_MASTER_URI exists in .bashrc, delete it
    sed -i '/export ROS_MASTER_URI=/d' ~/.bashrc
    echo "ROS_MASTER_URI has been removed from ~/.bashrc."
else
    echo "ROS_MASTER_URI is not set in ~/.bashrc."
fi


# ----- Delete ROS_IP -----

# Delete ROS_IP from ~/.bashrc
if grep -q "export ROS_IP=" ~/.bashrc; then
    # If ROS_IP exists in .bashrc, delete it
    sed -i '/export ROS_IP=/d' ~/.bashrc
    echo "ROS_IP has been removed from ~/.bashrc."
else
    echo "ROS_IP is not set in ~/.bashrc."
fi

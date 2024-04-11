#!/bin/bash

# Check if an IP address is provided as a command-line argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <ros-master-ip>"
    exit 1
fi


# ----- Connect to eduroam wifi -----
SSID=eduroam

# Check if the WiFi network is already known
if nmcli con show | grep -q "$SSID"; then
    echo "WiFi network $SSID is already known. Attempting to connect..."
    nmcli con up id "$SSID"
else
    echo "Adding new WiFi network $SSID."
    # Add the WiFi network and connect
    nmcli dev wifi connect "$SSID"
fi

if [ $? -eq 0 ]; then
    echo "Successfully connected to $SSID."
else
    echo "Failed to connect to $SSID."
fi


# ----- Set up the ROS_IP -----

# Specify the network interface, e.g., eth0, wlan0
NET_INTERFACE="wlan0"

# Get the IP address of the specified interface
IP_ADDR=$(hostname -I | awk '{print $1}')

# Export the IP address as ROS_IP
export ROS_IP=$IP_ADDR

# Update or add ROS_IP in ~/.bashrc
if grep -q "export ROS_IP=" ~/.bashrc; then
    # If ROS_IP exists in .bashrc, update it
    sed -i "/export ROS_IP=/c\export ROS_IP=$ROS_IP" ~/.bashrc
else
    # Otherwise, add it to .bashrc
    echo "export ROS_IP=$ROS_IP" >> ~/.bashrc
fi

# Print the ROS_IP for confirmation
echo "ROS_IP is set to $ROS_IP"




# ----- Set up ROS Master to the laptop -----



# The first command-line argument is the ROS master's IP address
ROS_MASTER_IP=$1

# Set the ROS_MASTER_URI environment variable
export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311

# Update or add ROS_MASTER_URI in ~/.bashrc
if grep -q "export ROS_MASTER_URI=" ~/.bashrc; then
    # If ROS_MASTER_URI exists in .bashrc, update it
    sed -i "/export ROS_MASTER_URI=/c\export ROS_MASTER_URI=$ROS_MASTER_URI" ~/.bashrc
else
    # Otherwise, add it to .bashrc
    echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc
fi

# Print the ROS_MASTER_URI for confirmation
echo "ROS_MASTER_URI is set to $ROS_MASTER_URI"

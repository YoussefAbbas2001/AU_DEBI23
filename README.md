# AU_DEBI23
This repo for DEBI competition 2023 



# INSTALL 
'''shell
sudo apt remove ros-noetic-dynamixel-sdk
sudo apt remove ros-noetic-turtlebot3-msgs
sudo apt remove ros-noetic-turtlebot3
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
'''


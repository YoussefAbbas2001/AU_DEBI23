# AU_DEBI23
This repo for DEBI competition 2023 

<img src="assets/images/DEBI_mao.jpeg" alt="MarineGEO circle logo" style="height: 700px; width:1200px;"/>


# INSTALL 

sudo apt remove ros-noetic-dynamixel-sdk <br>
sudo apt remove ros-noetic-turtlebot3-msgs <br>
sudo apt remove ros-noetic-turtlebot3<br>
mkdir -p ~/catkin_ws/src <br>
cd ~/catkin_ws/src/<br>
git clone https://github.com/YoussefAbbas2001/AU_DEBI23.git <br>
cd ~/catkin_ws && catkin_make<br>
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc<br>


# RUN
roslaunch AU_DEBI turtlebot3_DEBI_v1.launch <br>
rosrun AU_DEBI ball_shooting.py <br>

<img src="assets/images/camera_view.jpeg" alt="MarineGEO circle logo" style="height: 400px; width:600px;"/>
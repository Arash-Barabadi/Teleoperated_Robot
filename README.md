# Teleoperated_Auto
In a university course/project we as a student team focusing on building and testing a teleoperated robot.

# 1- In the first step we installed ROS2 on jetson Nano as follows:
## Dependencies 
### install additional dependencies
```bash
sudo apt install -y python3-colcon-common-extensions
```
## Creat a workspace in ROS2 to organaize our ROS2 packages:
```bash
      mkdir -p ~/teleauto_ws/src
      cd ~/teleauto_ws
```
## clone ROS2 repositories: 
### clone ROS2 repositories into the "src" directory;
```bash
wget https://raw.githubusercontent.com/ros2/ros2/release-foxy/ros2.repos
vcs import src < ros2.repos
```



vncviewr software 
sudo apt-get install gvncviewer

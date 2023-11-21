gvncviewer 192.168.55.1# Teleoperated_Auto
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
## Install and running procedure of gvncviewer (Virtual application for Virtual Network Computing):
```bash
sudo apt-get install gvncviewer
gvncviewer 192.168.55.1
Pass : abcd1234
```

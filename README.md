gvncviewer 192.168.55.1# Teleoperated_Auto
In a university course/project we as a student team focusing on building and testing a teleoperated robot.
# https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#installing-ros-2-and-its-utilities
## Install and running procedure of gvncviewer(virtual application for Virtual Network Computing):

```bash
sudo apt-get install gvncviewer
gvncviewer 192.168.55.1
Pass : abcd1234
```
## Creat a workspace and make a package ready into it.
```bash
mkdir -p ~/projekt1_ws/src
cd projekt1_ws
colcon build
```
## Next, we’ll clone the repo into the src directory of our workspace
```bash
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
```
## Then we’ll update the git submodules and pull in all the necessary packages
```bash
cd f1tenth_system
git submodule update --init --force --remote
```
## After git finishes cloning, we can now install all dependencies for our packages with rosdep:
```bash
cd $HOME/f1tenth_ws
rosdep update
rosdep install --from-paths src -i -y
```
## If the command "rosdep install --from-paths src -i -y" doesn't work:
You are missing the xacro and the diagnostic_updater packages.
You can easily install all the dependencies automatically by launching this command from the root folder of your colcon workspace:
rosdep install --from-paths src --ignore-src -r -y
or manually by using the command
$ sudo apt install ros-galactic-xacro ros-galactic-diagnostic-updater

Please note that the ZED ROS2 Wrapper has not been tested with Galactic, it has been designed to work with Foxy and Humble, the LTS distributions of ROS2.
## 

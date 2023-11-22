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
cd f1tenth_system
git submodule update --init --force --remote

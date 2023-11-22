gvncviewer 192.168.55.1# Teleoperated_Auto
In a university course/project we as a student team focusing on building and testing a teleoperated robot.

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


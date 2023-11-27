gvncviewer 192.168.55.1# Teleoperated_Auto
In a university course/project we as a student team focusing on building and testing a teleoperated robot.
# https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#installing-ros-2-and-its-utilities

## At first for building any packages we should install COLCON, therefore ;
```bash
sudo apt install python3-colcon-common-extensions
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
rosdep install --from-paths src -i -y --rosdistro foxy
``` 

## If the command "rosdep install --from-paths src -i -y" doesn't work:
You are missing the xacro and the diagnostic_updater packages.
You can easily install all the dependencies automatically by launching this command from the root folder of your colcon workspace:
rosdep install --from-paths src --ignore-src -r -y
or manually by using the command
$ sudo apt install ros-galactic-xacro ros-galactic-diagnostic-updater

Please note that the ZED ROS2 Wrapper has not been tested with Galactic, it has been designed to work with Foxy and Humble, the LTS distributions of ROS2.
## Lastly, after dependencies are installed, we can build our workspace again with the driver stack pacakge : 
```bash
colcon build --symlink-install
```

## config folder modification
### config folder contains 5 files with .yaml extension. For adjusting the steering wheel and motor configuration, we should change the parameters in the joy_teleop.yaml and vesc.yaml files, accordingly.

## The command for checking the connection bewteen VESC and Laptop (Jetson) : 
```bash
sudo chmod 777 /dev/ttyACM0
````

## The command for activating all mechanism 
````bash
ros2 launch f1tenth_stack bringup_launch.py
````

## The message? for testing the Steering wheel : 
```bash
ros2 topic echo /commands/servo/position
````
## The message? for testing driving motor movement
```bash
ros2 topic echo /commands/motor/speed
```

## At the end of the day we should send all our software packages to Jetson Orin NX. So we have to install and run the so called gvncviewer(virtual application for Virtual Network Computing):
```bash
sudo apt-get install gvncviewer
gvncviewer 192.168.55.1
Pass : abcd1234
```


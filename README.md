gvncviewer 192.168.55.1# Teleoperated_Auto
In a university course/project we as a student team focusing on building and testing a teleoperated robot.

## At first for building any packages we should install COLCON, therefore ;
```bash
sudo apt install python3-colcon-common-extensions
```

## For using auto completion feature, which is not enabled by default, we should source the script "colcon-argcomplete.bash". Add the following line to .bashrc file.

```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
# Create a ROS2 Workspace. 
## Set up a workspace and make a package ready into it.
```bash
mkdir -p ~/projekt1_ws/src
cd projekt1_ws
colcon build
```
### When the workspace is created, it contains four different folders :
#### 1-build
#### 2-install : it contains many files, but right now two of them are important for us, namely "setup.bash" & "local_setup.bash". 
##### "local_setup.bash" will simply source only projekt1_ws workspace (overlay workspace). When I source "local_setup.bash" script, I can use whatever i created in porjekt1_ws workspace.
##### "setup.bash" will source the projekt1_ws (overlay) pluse the global ROS2 intallation (underlay workspace), therefore to make things simpler the following command should be written in .bashrc file to source the "setup.bash".
```bash
source ~/projekt1_ws/install/setup.bash
```
#### 3-log
#### 4-src (which has been created with the command "mkdir -p ~/projekt1_ws/src"

# Create a ROS2 Package 
### To create a ROS2 node, the Package is needed. Packages will allow user to implement the code into reusable blocks. Each package is an independent unit. For example, we can have a package to handle the camera, another package is for steering wheel of our Auto. ***It should be noted, that all the packages must be generated in src folder.***
## 1-navigate to the src(source) directory of the ROS packages. 
```bash
cd ~/projekt1_ws/src
```
## 2-create a package simply by typing: ros2 pkg create "a package name" "package type" "dependencies"
### "package name" : Simply choose a package name
### "package type" : Add an argument to specify which kind of package is desired. In ROS2 there is a diffrence between a python package and c++ package.
### "dependencies" : Dependencies are simply the packages, this new package relies on. 
```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
## *An overview of the contents of generated src folder.* 
### 1-my_py_pkg folder: This folder has always the same name as the package. All of the python node will be placed in this folder. It already contains "__init__.py" file. It doesn't need to be changed right now.   
### 2-resource folder:
### 3-test folder:
### 4-package.xml: Each package folder has a package.xml file, which includes basically two sub-sections. The first one contains information about the package like name, version, description, email and license, which can be adjusted based on what the developer person want to release. The scond section is related to the dependencies (library) which will be used by the package like rclpy. 





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


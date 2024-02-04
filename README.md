# Teleoperiertes Fahren
In einem Hochschulprojekt konzentrierten wir uns als Studententeam auf den Bau und das Testen eines teleoperierten Autos.

## Als ersten Schritt zur Erstellung von Paketen sollten wir daher COLCON installieren;
```bash
sudo apt install python3-colcon-common-extensions
```
## !!! 
#### Bitte beachten Sie, dass wir die globale ROS2-Installation wie unten beschrieben vornehmen müssen:
```bash
source /opt/ros/foxy/setup.bash
```
#### Und eine lokale ROS2-Installation wie unten (wenn wir die Ros2-Knoten starten wollen, die in unserem lokalen Paket erstellt wurden): 
```bash
source ~/projekt1_ws/install/setup.bash
```
#### Um die Funktion der automatischen Vervollständigung zu nutzen, die standardmäßig nicht aktiviert ist, 
#### sollten wir das Skript "colcon-argcomplete.bash" erstellen. Fügen Sie die folgende Zeile zur .bashrc-Datei hinzu.

```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
# Einen ROS2-Workspace erstellen. 
## Richten Sie einen Workspace ein und stellen Sie dort ein Paket bereit.
```bash
mkdir -p ~/projekt1_ws/src
cd projekt1_ws
colcon build
```
### Wenn der Workspace erstellt wird, enthält er vier verschiedene Ordner :
#### 1-build
#### 2-install: Sie enthält viele Dateien, aber im Moment sind zwei davon für uns wichtig, nämlich "setup.bash" & "local_setup.bash". 
##### "local_setup.bash" wird einfach nur den Workspace projekt1_ws (Overlay-Workspace) sourcen. Wenn ich das Skript "local_setup.bash" verwende, kann ich alles verwenden, was ich im Arbeitsbereich von porjekt1_ws erstellt habe.
##### "setup.bash" wird das projekt1_ws (overlay) und die globale ROS2-Installation (underlay workspace) verwenden, daher sollte der Einfachheit halber der folgende Befehl in die .bashrc-Datei geschrieben werden, um die "setup.bash" zu verwenden.
```bash
source ~/projekt1_ws/install/setup.bash
```

#### 3-log
#### 4-src (das mit dem Befehl "mkdir -p ~/projekt1_ws/src" erstellt wurde)

# Ein ROS2-Paket(Package) erstellen 
### Um einen ROS2-Knoten zu erstellen, wird ein Paket benötigt. Pakete ermöglichen es dem Benutzer, den Code in wiederverwendbare Blöcke zu implementieren. Jedes Paket ist eine unabhängige Einheit. Zum Beispiel können wir ein Paket haben, um die Kamera zu handhaben, ein anderes Paket ist für das Lenkrad unseres Autos.***Es sollte beachtet werden, dass alle Pakete im src-Ordner erzeugt werden müssen.***
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
## ***An overview of the contents of generated src folder.*** 
#### 1-my_py_pkg folder: This folder has always the same name as the package. All of the python nodes will be placed in this folder. It already contains "__init__.py" file. It doesn't need to be changed right now.   
#### 2-resource folder:
#### 3-test folder:
#### 4-package.xml: Each package folder has a package.xml file, which includes basically two sub-sections. The first one contains information about the package like name, version, description, email and license, which can be adjusted based on what the developer person want to release. The second section is related to the dependencies (library) which will be used by the package like rclpy. ***If the user want to add new dependencies it should be written here as well.*** At the end of the file, the type of the package ,which is here ament_python, can be seen. 
#### 5-setup.cfg: 
#### 6-setup.py:

## Go back to the workspace address afterwards and type colcon build again, to confirm the creation of the new package "my_py_pkg"

```bash
colcon build
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.23s]          

Summary: 1 package finished [3.09s]
```
## Or if there are many packages, the following command can be written specifically for one package, therefore the compile time should be shorter.
```bash
colcon build --packages-select my_py_pkg
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.26s]          

Summary: 1 package finished [3.13s]
```
## Now the python package is ready to host any python node.
#
# Create a Node
## A node is a subprogram in an application, responsible for only one thing.
## Nodes are combined into a graph and communicate with each other through topics, services and parameters.
#
#
# Connecting IMU to Jetson NX Orin 16Gb
### Ground=GND port 14
### POWER=VDD port 16
### RX= Data sender port
### TX=  Data reciever port 
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


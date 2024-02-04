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
#### 2-installieren: Sie enthält viele Dateien, aber im Moment sind zwei davon für uns wichtig, nämlich "setup.bash" & "local_setup.bash". 
##### "local_setup.bash" wird einfach nur den Workspace projekt1_ws (Overlay-Workspace) sourcen. Wenn ich das Skript "local_setup.bash" verwende, kann ich alles verwenden, was ich im Arbeitsbereich von porjekt1_ws erstellt habe.
##### "setup.bash" wird das projekt1_ws (overlay) und die globale ROS2-Installation (underlay workspace) verwenden, daher sollte der Einfachheit halber der folgende Befehl in die .bashrc-Datei geschrieben werden, um die "setup.bash" zu verwenden.
```bash
source ~/projekt1_ws/install/setup.bash
```

#### 3-log
#### 4-src (das mit dem Befehl "mkdir -p ~/projekt1_ws/src" erstellt wurde)

# Ein ROS2-Paket(Package) erstellen 
### Um einen ROS2-Knoten zu erstellen, wird ein Paket benötigt. Pakete ermöglichen es dem Benutzer, den Code in wiederverwendbare Blöcke zu implementieren. Jedes Paket ist eine unabhängige Einheit. Zum Beispiel können wir ein Paket haben, um die Kamera zu handhaben, ein anderes Paket ist für das Lenkrad unseres Autos.***Es sollte beachtet werden, dass alle Pakete im src-Ordner erzeugt werden müssen.***
## 1. Navigiert wird zum src-verzeichnis der ROS-Pakete.
```bash
cd ~/projekt1_ws/src
```
## 2. Ein Paket wird einfach durch Eingabe erstellt: ros2 pkg create "Paketname" "Pakettyp" "Abhängigkeiten"
### "Paketname": Wählen Sie einfach einen Paketnamen
### "Pakettyp": Geben Sie ein Argument an, um anzugeben, welcher Art das Paket sein soll. In ROS2 gibt es einen Unterschied zwischen einem Python-Paket und einem C++-Paket.
### "Abhängigkeiten": Abhängigkeiten sind einfach die Pakete, von denen dieses neue Paket abhängt.
```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
## ***Eine Übersicht über den Inhalt des generierten src-Ordners.*** 
#### 1-mein_py_pkg Ordner: Dieser Ordner hat immer den gleichen Namen wie das Paket. Alle Python-Knoten werden in diesem Ordner abgelegt. Er enthält bereits die Datei "__init__.py". Sie braucht im Moment nicht geändert zu werden.   
#### 2-resource folder:
#### 3-test folder:
#### 4-package.xml: Jeder Paket-Ordner hat eine package.xml-Datei, die im Wesentlichen zwei Unterabschnitte enthält. Die erste enthält Informationen über das Paket wie Name, Version, Beschreibung, E-Mail und Lizenz, die je nach dem, was der Entwickler veröffentlichen möchte, angepasst werden können. Der zweite Abschnitt bezieht sich auf die Abhängigkeiten (Bibliotheken), die von dem Paket wie rclpy verwendet werden. ***Wenn der Benutzer neue Abhängigkeiten hinzufügen möchte, sollte dies ebenfalls hier eingetragen werden.*** Am Ende der Datei ist der Typ des Pakets zu sehen, der hier ament_python ist. 
#### 5-setup.cfg: 
#### 6-setup.py:

## Gehen Sie danach zur Arbeitsbereichsadresse zurück und geben Sie erneut "colcon build" ein, um die Erstellung des neuen Pakets "my_py_pkg" zu bestätigen.

```bash
colcon build
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.23s]          

Summary: 1 package finished [3.09s]
```
## Oder wenn es viele Pakete gibt, kann der folgende Befehl speziell für ein Paket geschrieben werden, wodurch die Kompilierzeit kürzer sein sollte.
```bash
colcon build --packages-select my_py_pkg
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.26s]          

Summary: 1 package finished [3.13s]
```
## Jetzt ist das Python-Paket bereit, jeden Python-Knoten zu hosten.

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

## The message for testing the Steering wheel : 
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


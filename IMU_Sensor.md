# Inertia measurement unit (IMU) senosr
## I'm going to make a node, which subscribes to sensor data (from the bank file) and publish some part of that data

### 1- Create a package as below;
#### rclpy is the first library that the dependency to it should be adjusted at first. More dependcies can be added afterwards.
```bash
ros2 pkg create imu --build-type ament_python --dependencies rclpy
```
### 2- Create the publisher & subscriber python nodes. 

#### in the python node the message type should be determined and reffered. In ROS 2, the sensor-related messages are typically part of the "sensor_msgs" package, just like in ROS 1. The "sensor_msgs" package defines standard message types for sensor data and is commonly used in ROS 2 applications involving sensors.

#### therefore it should be called at first as bellow: 
```python
from sensor_msgs.msg import Imu
```


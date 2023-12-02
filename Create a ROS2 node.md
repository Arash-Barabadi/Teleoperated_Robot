#!/usr/bin/env python3

#For using ROS2 functionalities, the rclpy library has been imported.
import rclpy
from rclpy.node import Node 

class MyNode(Node) : 

    def __init__(self):
        #The following line will call __init__ function from the Node.
        super().__init__("py_test")
        # instead of node.get_logger() we are gonna use the below line : 
        self.get_logger().info("Hello ROS2")


def main(args=None):
    #For starting the ROS2 communication the following line must be written. 
    rclpy.init(args=args)

    #To construct the node, we should use Node class as an object, and insert the node name as a parameter. 
    #Note that, the name of the node is not the name of the file. 
    ## node = Node("py_test")  !!instead of creating Node objet. I will create MyNode()
    node = MyNode()

    #Let's print something with get_logger. So you will use get_logger function from the node objet.
    ## node.get_logger().info("Hello ROS2")

    #The following line is used to make your node be alive. The callbacks will be able to be called from spin function.
    rclpy.spin(node)

    #For ending the ROS2 communication the following line must be written.
    rclpy.shutdown()


if __name__ == "__main__" : 
    main()
## Hey

### since this node does nothing about printing a log. Let's use one of the most basic and common functionalities in ROS, which is a timer. A timer will allow to call a function with a given rate. A function is needed to be called at 10Hz, for instance, therefore the timer comes into play. 

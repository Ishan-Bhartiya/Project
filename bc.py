#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class RoverCommander(Node):
    def __init__(self):
        super().__init__('rover_commander')
        self.pub = self.create_publisher(Float64MultiArray, '/rover/control_input', 10)

    def send(self, data, duration):
        data = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0] 
        msg = Float64MultiArray()
        msg.data = data
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.pub.publish(msg)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = RoverCommander()
    # Move forward
    node.send([0,0,0,0,1,1,1,1], 2)
    # Stop and steer
    node.send([-1.5708]*4 + [0]*4, 1)
    # Move sideways
    node.send([-1.5708]*4 + [1]*4, 2)
    # Stop
    node.send([-1.5708]*4 + [0]*4, 1)
    # Differential turn
    node.send([0]*4 + [-1,1,-1,1], 2)
    # Point turn
    node.send([0.7854,0.7854,-0.7854,-0.7854,1,1,1,1], 2)
    # 45 degree move
    node.send([0.7854]*4 + [1]*4, 2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



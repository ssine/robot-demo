#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import Joy
from .key_parser import get_key, save_terminal_settings, restore_terminal_settings

class KeyJoyNode(Node):
    def __init__(self):
        super().__init__('mpi_controller')
        self.settings = save_terminal_settings()
        self.pub_joy = self.create_publisher(Joy, '/joy', qos_profile_system_default)
        self.run()

    def run(self):
        while True:
            # parse keyboard control
            print('before get')
            key = get_key(self.settings, timeout=0.1)
            print('after get', key)

            # interpret keyboard control as joy
            joy_msg, flag = self.key_to_joy(key)
            if flag is False:
                break

            # publish joy
            self.pub_joy.publish(joy_msg)
    
        self.stop()

    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if key == 'w':
            joy_msg.axes[1] = 1.0
        elif key == 's':
            joy_msg.axes[1] = -1.0
        elif key == 'a':
            joy_msg.axes[0] = -1.0
        elif key == 'd':
            joy_msg.axes[0] = 1.0
        elif key == 'q':
            joy_msg.axes[2] = -1.0
        elif key == 'e':
            joy_msg.axes[2] = 1.0
        elif (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
            flag = False
        return joy_msg, flag
    

    def stop(self):
        restore_terminal_settings(self.settings)

def main():
    rclpy.init()
    key_joy_node = KeyJoyNode()
    rclpy.spin(key_joy_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

""" MegaPi Controller ROS Wrapper"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist
from .mpi_control import MegaPiController
import numpy as np


class MegaPiControllerNode(Node):

  def __init__(self, verbose=True, debug=False):
    super().__init__('mpi_twist_controller')
    self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
    self.create_subscription(Twist, '/twist', self.twist_callback, qos_profile_system_default)
    self.r = 0.025  # radius of the wheel
    self.lx = 0.055  # half of the distance between front wheel and back wheel
    self.ly = 0.07  # half of the distance between left wheel and right wheel
    self.calibration = 210.0

  def twist_callback(self, twist_cmd):
    desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
    # calculate the jacobian matrix
    jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)], [1, 1,
                                                                (self.lx + self.ly)], [1, 1, -(self.lx + self.ly)],
                                [1, -1, (self.lx + self.ly)]]) / self.r
    # calculate the desired wheel velocity
    result = np.dot(jacobian_matrix, desired_twist)

    # send command to each wheel
    self.mpi_ctrl.setFourMotors(int(result[0][0]), int(result[1][0]), int(result[2][0]), int(result[3][0]))


def main():
  rclpy.init()
  mpi_ctrl_node = MegaPiControllerNode()
  rclpy.spin(mpi_ctrl_node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()

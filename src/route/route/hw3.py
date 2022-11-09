#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import time
import math
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Joy

speed_rclock = 1.64
speed_clock = 1.64
speed_forward = 0.42
speed_backward = 0.42
speed_slide = 0.28
eps = 1e-2


def get_msg(action):
  joy_msg = Joy()
  joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
  if action == 'rclock':
    joy_msg.axes[2] = 1.0
  elif action == 'clock':
    joy_msg.axes[2] = -1.0
  elif action == 'forward':
    joy_msg.axes[1] = 1.0
  elif action == 'backward':
    joy_msg.axes[1] = -1.0
  elif action == 'left':
    joy_msg.axes[0] = 1.0
  elif action == 'right':
    joy_msg.axes[0] = -1.0
  return joy_msg


def normalize_dr(dr):
  while dr < -math.pi:
    dr += 2 * math.pi
  while dr > math.pi:
    dr -= 2 * math.pi
  return dr


class Route(Node):

  def __init__(self):
    super().__init__('route_planner')
    self.pub_joy = self.create_publisher(Joy, "/joy", qos_profile_system_default)
    self.state = {
        'x': 0,
        'y': 0,
        'r': 0,
    }

  def to_orientation(self, r):
    dr = normalize_dr(r - self.state['r'])
    if dr < 0:
      self.pub_joy.publish(get_msg('clock'))
      time.sleep(-dr / speed_clock)
    else:
      self.pub_joy.publish(get_msg('rclock'))
      time.sleep(dr / speed_rclock)
    self.pub_joy.publish(get_msg('stop'))
    self.state['r'] = r

  def move_line(self, distance):
    if distance > 0:
      self.pub_joy.publish(get_msg('forward'))
      time.sleep(distance / speed_forward)
    else:
      self.pub_joy.publish(get_msg('backward'))
      time.sleep(-distance / speed_backward)
    self.pub_joy.publish(get_msg('stop'))

  def to_state(self, new_state):
    dx = new_state['x'] - self.state['x']
    dy = new_state['y'] - self.state['y']

    r = math.atan2(dy, dx)
    if abs(r) > eps:
      self.to_orientation(r)

    d = math.sqrt(dx**2 + dy**2)
    if d > eps:
      self.move_line(d)

    self.to_orientation(new_state['r'])
    self.state = new_state
    print(self.state)

  def run(self):
    unit = 0.5
    self.to_state({'x': 1 * unit, 'y': 0 * unit, 'r': 0})
    time.sleep(1)
    self.to_state({'x': 1 * unit, 'y': 2 * unit, 'r': math.pi})
    time.sleep(1)

    # self.pub_joy.publish(get_msg('rclock'))
    # time.sleep(0.1 / speed_rclock)
    # self.pub_joy.publish(get_msg('stop'))
    # time.sleep(0.3)
    self.pub_joy.publish(get_msg('backward'))
    time.sleep(0.1 / speed_backward)
    self.pub_joy.publish(get_msg('stop'))

    time.sleep(1)
    self.to_state({'x': 0 * unit, 'y': 0 * unit, 'r': 0})
    time.sleep(1)

    self.pub_joy.publish(get_msg('rclock'))
    time.sleep(0.1 / speed_rclock)
    self.pub_joy.publish(get_msg('stop'))
    time.sleep(0.3)
    self.pub_joy.publish(get_msg('backward'))
    time.sleep(0.05 / speed_backward)
    self.pub_joy.publish(get_msg('stop'))


def main():
  rclpy.init()
  key_joy_node = Route()
  key_joy_node.run()
  # rclpy.spin(key_joy_node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()
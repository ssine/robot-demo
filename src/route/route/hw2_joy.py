#!/usr/bin/env python
import math
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from .pid import PIDcontroller
from interface.msg import AprilTagDetection, AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs

# x, y, z, y, p, r
tag_positions = {
    0: [1.0, 0.0, 0.11, math.pi, -math.pi / 2, 0],
    1: [0.5, 1.685, 0.2, -math.pi / 2, 0, 0],
    2: [-0.5, 1.0, 0.107, 0, -math.pi / 2, 0]
}

speed_rclock = 1.4
speed_clock = 1.4
speed_forward = 0.4
speed_backward = 0.4
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
    joy_msg.axes[0] = -1.0
  elif action == 'right':
    joy_msg.axes[0] = 1.0
  return joy_msg


def normalize_dr(dr):
  while dr < -math.pi:
    dr += 2 * math.pi
  while dr > math.pi:
    dr -= 2 * math.pi
  return dr


def getError(currentState, targetState):
  """
      return the different between two states
      """
  result = targetState - currentState
  result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
  return result


def inv_pose(pose: Pose):
  # inverse the position
  pose.position.x = -pose.position.x
  pose.position.y = -pose.position.y
  pose.position.z = -pose.position.z
  # to inverse a quaternion, negate the w. see http://wiki.ros.org/tf2/Tutorials/Quaternions
  pose.orientation.w = -pose.orientation.w
  return pose


class RoutePlanner(Node):

  def __init__(self, waypoints):
    super().__init__('route_planner')
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    self.pub_joy = self.create_publisher(Joy, "/joy", qos_profile_system_default)
    self.create_subscription(
        AprilTagDetectionArray, "/apriltag_detection_array", self.on_detection,
        QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                   history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                   depth=1))
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.position = np.array([0.0, 0.0, 0.0])
    self.broadcast_bot_position()
    self.last_action = 'run'
    self.broadcast_initial_positions()
    self.waypoints = waypoints
    self.timer = self.create_timer(0.5, self.tick)

  def tick(self):
    self.broadcast_bot_position()
    if len(self.waypoints) < 1:
      return
    wp = self.waypoints[0]

    # if self.last_action == 'run':
    # wait for calibration
    # return

    # check and switch waypoint
    if np.linalg.norm(getError(self.position, wp)) < 0.05:
      print(f'waypoint {wp} has reached')
      self.waypoints = self.waypoints[1:]
      if len(self.waypoints) < 1:
        print('no more waypoints, stopping')
        self.pub_joy.publish(get_msg('stop'))
        # self.destroy_node()
        return
      wp = self.waypoints[0]
      print(f'next waypoint: {wp}')

    self.to_state(wp)
    self.last_action = 'run'

  def on_detection(self, tags_msg: AprilTagDetectionArray):
    return
    if self.last_action == 'calibration':
      return
    estimated_positions = []
    for detection in tags_msg.detections:
      if detection.id not in tag_positions:
        continue
      est_pose = self.transform_pose(inv_pose(detection.pose), f'tag_{detection.id}', 'map')
      x = est_pose.position.x
      y = est_pose.position.y
      _, _, w = euler_from_quaternion(
          [est_pose.orientation.x, est_pose.orientation.y, est_pose.orientation.z, est_pose.orientation.w])
      estimated_positions.append(np.array([x, y, w + math.pi / 2]))
    if len(estimated_positions) > 0:
      self.position = np.mean(estimated_positions, axis=0)
      self.last_action = 'calibration'
      print('calibrated position: ', self.position)

  def to_orientation(self, r):
    dr = normalize_dr(r - self.position[2])
    if dr < 0:
      self.pub_joy.publish(get_msg('clock'))
      time.sleep(-dr / speed_clock)
    else:
      self.pub_joy.publish(get_msg('rclock'))
      time.sleep(dr / speed_rclock)
    self.pub_joy.publish(get_msg('stop'))
    self.position[2] = r

  def move_line(self, distance):
    if distance > 0:
      self.pub_joy.publish(get_msg('forward'))
      time.sleep(distance / speed_forward)
    else:
      self.pub_joy.publish(get_msg('backward'))
      time.sleep(-distance / speed_backward)
    self.pub_joy.publish(get_msg('stop'))

  def to_state(self, new_state):
    dx = new_state[0] - self.position[0]
    dy = new_state[1] - self.position[1]

    r = math.atan2(dy, dx)
    if abs(r) > eps:
      self.to_orientation(r)

    d = math.sqrt(dx**2 + dy**2)
    if d > eps:
      self.move_line(d)

    self.to_orientation(new_state[2])
    self.position = new_state
    print(self.position)

  def broadcast_bot_position(self):
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'bot'

    t.transform.translation.x = float(self.position[0])
    t.transform.translation.y = float(self.position[1])
    t.transform.translation.z = 0.0
    quat = quaternion_from_euler(0, 0, self.position[2] - math.pi / 2)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    self.tf_static_broadcaster.sendTransform(t)

  def broadcast_initial_positions(self):
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'bot'
    t.child_frame_id = 'camera_0'

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.125
    quat = quaternion_from_euler(-math.pi / 2, 0, 0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    self.tf_static_broadcaster.sendTransform(t)

    for tag_id in tag_positions.keys():
      t = TransformStamped()

      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'map'
      t.child_frame_id = f'tag_{tag_id}'

      p = tag_positions[tag_id]

      t.transform.translation.x = float(p[0])
      t.transform.translation.y = float(p[1])
      t.transform.translation.z = float(p[2])
      quat = quaternion_from_euler(float(p[3]), float(p[4]), float(p[5]))
      t.transform.rotation.x = quat[0]
      t.transform.rotation.y = quat[1]
      t.transform.rotation.z = quat[2]
      t.transform.rotation.w = quat[3]

      self.tf_static_broadcaster.sendTransform(t)

  def transform_pose(self, input_pose, from_frame, to_frame):
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = self.get_clock().now().to_msg()
    try:
      output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame)
      return output_pose_stamped.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      raise


def main():
  rclpy.init()
  node = RoutePlanner(np.array([
      [0.0, 0.0, 0.0],
      [0.5, 0.0, 0.0],
      [0.5, 1.0, np.pi],
      [0.0, 0.0, 0.0],
  ]))
  try:
    rclpy.spin(node)
  except:
    node.pub_joy.publish(get_msg('stop'))
    raise
  rclpy.shutdown()


if __name__ == "__main__":
  main()

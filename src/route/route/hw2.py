#!/usr/bin/env python
import math
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TransformStamped
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


def genTwistMsg(desired_twist):
  """
    Convert the twist to twist msg.
    """
  twist_msg = Twist()
  twist_msg.linear.x = desired_twist[0]
  twist_msg.linear.y = desired_twist[1]
  twist_msg.linear.z = 0.0
  twist_msg.angular.x = 0.0
  twist_msg.angular.y = 0.0
  twist_msg.angular.z = desired_twist[2]
  return twist_msg


def coord(twist, current_state):
  J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0], [0.0, 0.0, 1.0]])
  return np.dot(J, twist)


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
    self.pid = PIDcontroller(0.015, 0.006, 0.005)
    self.pub_twist = self.create_publisher(Twist, "/twist", qos_profile_system_default)
    self.create_subscription(AprilTagDetectionArray, "/apriltag_detection_array", self.on_detection,
                             qos_profile_system_default)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.position = np.array([0.0, 0.0, 0.0])  # x, y, w
    self.target = np.array([0.0, 0.0, 0.0])
    self.last_calibrated_position = np.array([0.0, 0.0, 0.0])
    self.tn = 0
    self.broadcast_bot_position()
    self.broadcast_initial_positions()
    self.waypoints = waypoints
    self.tick()
    self.timer = self.create_timer(self.pid.timestep, self.tick)

  def tick(self):
    self.broadcast_bot_position()
    if len(self.waypoints) < 1:
      return
    wp = self.waypoints[0]

    # check and switch waypoint
    print('tn = ', self.tn)
    if np.linalg.norm(self.pid.getError(self.position, wp)) < 0.05:
      print(f'waypoint {wp} has reached')
      self.waypoints = self.waypoints[1:]
      if len(self.waypoints) < 1:
        print('no more waypoints, stopping')
        self.pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
        # self.destroy_node()
        return
      wp = self.waypoints[0]
      print(f'next waypoint: {wp}')
      self.pid.setTarget(wp)

    update_value = self.pid.update(self.position)
    c = coord(update_value, self.position)
    self.tn = np.linalg.norm(c)
    self.pub_twist.publish(genTwistMsg(c))
    self.position += update_value
    print('updated position: ', self.position)

  def on_detection(self, tags_msg: AprilTagDetectionArray):
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
      self.last_calibrated_position[:] = self.position[:]
      print('calibrated position: ', self.position)

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
    node.pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
    raise
  rclpy.shutdown()


if __name__ == "__main__":
  main()

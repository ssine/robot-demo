#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Joy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from interface.msg import AprilTagDetection, AprilTagDetectionArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist, Pose
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs
from .kalman_slam import KalmanSLAM

speed_rclock = 1
speed_forward = 0.3
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


def inv_pose(pose: Pose):
  # inverse the position
  pose.position.x = -pose.position.x
  pose.position.y = -pose.position.y
  pose.position.z = -pose.position.z
  # to inverse a quaternion, negate the w. see http://wiki.ros.org/tf2/Tutorials/Quaternions
  pose.orientation.w = -pose.orientation.w
  return pose


class Route(Node):

  def __init__(self):
    super().__init__('route_planner')
    init_state = np.zeros(3)
    # square
    self.controls = [
        ['backward', 0.5 / speed_forward],
        ['rclock', (math.pi / 2) / speed_rclock],
        ['backward', 0.5 / speed_forward],
        ['rclock', (math.pi / 2) / speed_rclock],
        ['backward', 0.5 / speed_forward],
        ['rclock', (math.pi / 2) / speed_rclock],
        ['backward', 0.5 / speed_forward],
        ['rclock', (math.pi / 2) / speed_rclock],
    ]
    # octagon
    # init_state[2] = -0.3927
    # self.controls = [
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    #     ['backward', 0.2706 / speed_forward],
    #     ['rclock', (math.pi / 4) / speed_rclock],
    # ]
    self.state = init_state
    self.slam = KalmanSLAM(init_state.copy())
    self.last_ctrl_time = -1
    self.last_tick_time = -1
    self.last_measure_time = -1
    self.measurement = {}

    # publish control and receive tags
    self.pub_joy = self.create_publisher(Joy, "/joy", qos_profile_system_default)
    self.create_subscription(AprilTagDetectionArray, "/apriltag_detection_array", self.on_detection,
                             qos_profile_system_default)

    # poses
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.broadcast_bot_position()
    self.broadcast_initial_positions()

    self.timer = self.create_timer(0.05, self.tick)

    self.dumped = False

  def tick(self):
    if len(self.controls) < 1:
      self.pub_joy.publish(get_msg('stop'))
      if not self.dumped:
        self.slam.export_state('st.pkl')
        self.dumped = True
      return
    self.broadcast_bot_position()
    ctl = self.controls[0]
    now = time.time()

    # issue initial control
    if self.last_ctrl_time == -1:
      self.pub_joy.publish(get_msg(ctl[0]))
      self.pub_joy.publish(get_msg(ctl[0]))
      self.pub_joy.publish(get_msg(ctl[0]))
      self.last_ctrl_time = now
      self.last_tick_time = now
      return

    # update state
    dt = now - self.last_tick_time
    control = np.zeros(3)
    if ctl[0] == 'backward':
      control[0] = speed_forward * dt * np.cos(self.state[2])
      control[1] = speed_forward * dt * np.sin(self.state[2])
    elif ctl[0] == 'rclock':
      control[2] = (speed_rclock + 0.2) * dt
    self.slam.step(control)
    if now - self.last_measure_time < 0.07:
      # make sure the measurement is valid
      self.slam.update(self.measurement)
    self.state = self.slam.get_bot_state()
    self.broadcast_bot_position()
    self.broadcast_tag_positions()
    # print('update result: ', self.state, self.slam.get_tag_status())

    # issue next control
    if now - self.last_ctrl_time >= ctl[1]:
      self.controls = self.controls[1:]
      if len(self.controls) > 0:
        ctl = self.controls[0]
        self.pub_joy.publish(get_msg(ctl[0]))
        self.last_ctrl_time = now

    self.last_tick_time = time.time()

  def on_detection(self, tags_msg: AprilTagDetectionArray):
    tag_dets = {}
    for detection in tags_msg.detections:
      pose = self.transform_pose(detection.pose, 'camera_0', 'map')
      x = pose.position.x
      y = pose.position.y
      _, _, w = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
      tag_dets[detection.id] = [x, y, w]
    self.last_measure_time = time.time()
    self.measurement['tags'] = tag_dets
    self.measurement['bot'] = self.state
    try:
      bot_pose_estimates = []
      for detection in tags_msg.detections:
        est_pose = self.transform_pose(inv_pose(detection.pose), f'slam_tag_{detection.id}', 'map')
        x = est_pose.position.x
        y = est_pose.position.y
        _, _, w = euler_from_quaternion(
            [est_pose.orientation.x, est_pose.orientation.y, est_pose.orientation.z, est_pose.orientation.w])
        bot_pose_estimates.append(np.array([x, y, w + math.pi / 2]))
        # hack: reduce measurement impact as control signal is well enough
        self.measurement['bot'] = self.state * 0.8 + np.mean(bot_pose_estimates, axis=0) * 0.2
    except Exception as err:
      print('no slam pose available, skip bot position inference.')
    # print(self.measurement)

  def broadcast_tag_positions(self):
    tag_status = self.slam.get_tag_status()
    stamp = self.get_clock().now().to_msg()
    for tag_id, tag_state in tag_status.items():
      t = TransformStamped()

      t.header.stamp = stamp
      t.header.frame_id = 'map'
      t.child_frame_id = f'slam_tag_{tag_id}'

      t.transform.translation.x = float(tag_state[0])
      t.transform.translation.y = float(tag_state[1])
      t.transform.translation.z = 0.0
      quat = quaternion_from_euler(0, 0, tag_state[2] - math.pi / 2)
      t.transform.rotation.x = quat[0]
      t.transform.rotation.y = quat[1]
      t.transform.rotation.z = quat[2]
      t.transform.rotation.w = quat[3]

      self.tf_static_broadcaster.sendTransform(t)

  def broadcast_bot_position(self):
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'bot'

    t.transform.translation.x = float(self.state[0])
    t.transform.translation.y = float(self.state[1])
    t.transform.translation.z = 0.0
    quat = quaternion_from_euler(0, 0, self.state[2] - math.pi / 2)
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
  key_joy_node = Route()
  rclpy.spin(key_joy_node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()
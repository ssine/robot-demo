import cv2
import numpy as np
import apriltag
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf_transformations import quaternion_from_matrix
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import TransformStamped
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from interface.msg import AprilTagDetection, AprilTagDetectionArray

options = apriltag.DetectorOptions(families='tag36h11')
detector = apriltag.Detector(options)

mtx = np.array([[1.40195505e+03, 0.00000000e+00, 9.37821211e+02], [0.00000000e+00, 1.40370812e+03, 5.18206208e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[2.51876482e-01, -7.28886391e-01, -2.36489737e-04, -1.03891640e-03, 7.06549536e-01]])


class AprilDetectorNode(Node):

  def __init__(self):
    super().__init__('april_detector')
    self.bridge = CvBridge()
    self.detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    self.caster = TransformBroadcaster(self)
    self.create_subscription(Image, '/camera_0', self.on_image, qos_profile_system_default)
    self.pose_pub = self.create_publisher(PoseArray, '/april_poses', qos_profile_system_default)
    self.det_pub = self.create_publisher(AprilTagDetectionArray, '/apriltag_detection_array',
                                         qos_profile_system_default)

  def on_image(self, message: Image):
    image = self.bridge.imgmsg_to_cv2(message)

    # undistort camera image
    h, w = image.shape[:2]
    n_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    image = cv2.undistort(image, mtx, dist, None, n_mtx)

    # detect april tags
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = self.detector.detect(grey)

    pose_msgs = []
    detection_msgs = []

    for detection in detections:
      # calculate pose
      # tag size: whole picture in meters
      M, _, _ = self.detector.detection_pose(detection, [n_mtx[0][0], n_mtx[1][1], n_mtx[0][2], n_mtx[1][2]],
                                             tag_size=0.2)
      t = M[:3, 3:]
      q = quaternion_from_matrix(M)

      # publish transform
      tfs = TransformStamped()
      tfs.header.stamp = self.get_clock().now().to_msg()
      tfs.header.frame_id = 'camera_0'
      tfs.child_frame_id = f'det_tag_{detection.tag_id}'
      tfs.transform.translation = Vector3(x=float(t[0]), y=float(t[1]), z=float(t[2]))
      tfs.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
      self.caster.sendTransform(tfs)

      # publish pose and april detection results
      pose = Pose()
      pose.position.x = float(t[0])
      pose.position.y = float(t[1])
      pose.position.z = float(t[2])
      pose.orientation = tfs.transform.rotation
      pose_msgs.append(pose)

      det = AprilTagDetection()
      det.header = message.header
      det.id = detection.tag_id
      det.pose = pose
      for i in range(4):
        det.corners2d[i].x = detection.corners[i][0]
        det.corners2d[i].y = detection.corners[i][1]
      detection_msgs.append(det)
    self.pose_pub.publish(PoseArray(header=message.header, poses=pose_msgs))
    self.det_pub.publish(AprilTagDetectionArray(header=message.header, detections=detection_msgs))


def main():
  rclpy.init()
  april_node = AprilDetectorNode()
  rclpy.spin(april_node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header

fps = 30
gst_pipeline = f'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate={fps}/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink'


class CameraNode(Node):

  def __init__(self):
    super().__init__('camera')
    self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    self.pub_cam = self.create_publisher(Image, '/camera_0', qos_profile_system_default)
    self.pub_cam_comp = self.create_publisher(CompressedImage, '/camera_0/compressed', qos_profile_system_default)
    self.bridge = CvBridge()
    self.timer = self.create_timer(1 / fps, self.tick)
    self.idx = 0

  def tick(self):
    self.idx += 1
    ret, frame = self.cap.read()
    if ret:
      header = Header(frame_id='camera_0')
      # msg = self.bridge.cv2_to_imgmsg(frame, header=header)
      # print(msg)
      self.pub_cam.publish(self.bridge.cv2_to_imgmsg(frame, header=header))
      # self.pub_cam_comp.publish(self.bridge.cv2_to_compressed_imgmsg(frame))
      # if self.idx % 100 == 0:
      #   cv2.imwrite('t.png', frame)
      #   print('sample written to t.png')


def main():
  rclpy.init()
  camera_node = CameraNode()
  rclpy.spin(camera_node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()

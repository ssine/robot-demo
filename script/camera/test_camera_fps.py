import cv2
import time

gst_pipeline = 'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink'

if __name__ == '__main__':
  video = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

  fps = video.get(cv2.CAP_PROP_FPS)
  print(f'Frames per second using video.get(cv2.CAP_PROP_FPS) : {fps}')

  num_frames = 120
  print(f'Capturing {num_frames} frames')

  start = time.time()
  for i in range(0, num_frames):
    ret, frame = video.read()
    print(frame.shape)
    # cv2.imshow('f', ret)
    # cv2.waitKey(5)
  end = time.time()

  seconds = end - start
  print(f'Time taken : {seconds} seconds')

  fps = num_frames / seconds
  print(f'Estimated frames per second : {fps}')

  video.release()

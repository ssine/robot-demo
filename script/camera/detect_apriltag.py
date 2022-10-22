import cv2
import apriltag

fps = 30
gst_pipeline = f'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate={fps}/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink'

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
options = apriltag.DetectorOptions(families='tag36h11')
detector = apriltag.Detector(options)

while True:
  ret, frame = cap.read()

  if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break

  # display video in low quality for the sake of x11 forwarding
  grey = cv2.cvtColor(cv2.resize(frame, (640, 360)), cv2.COLOR_BGR2GRAY)
  image = cv2.resize(frame, (640, 360))
  results = detector.detect(grey)

  # loop over the AprilTag detection results
  for r in results:
    # extract the bounding box (x, y)-coordinates for the AprilTag
    # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)
    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    # draw the tag family on the image
    tagFamily = r.tag_family.decode("utf-8")
    cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print("[INFO] tag family: {}".format(tagFamily))

  cv2.imshow('cap', image)

  key = cv2.waitKey(1)

  if key == ord('q'):
    break

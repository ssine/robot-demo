import cv2

img_folder = './script/camera/data/'

fps = 30
gst_pipeline = f'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate={fps}/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink'

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
idx = 0

while True:
  ret, frame = cap.read()

  if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break

  # display video in low quality for the sake of x11 forwarding
  disp = cv2.flip(cv2.cvtColor(cv2.resize(frame, (640, 360)), cv2.COLOR_BGR2GRAY), 1)
  cv2.imshow('cap', disp)

  key = cv2.waitKey(1)

  if key == ord('q'):
    break
  if key == ord('c'):
    fn = img_folder + f'image_{idx}.png'
    cv2.imwrite(fn, frame)
    print(f'{fn} captured')
    idx += 1

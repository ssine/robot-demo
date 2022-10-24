# ROS2 Robot Demo

## Setup

Modify user id and workspace in `script/Dockerfile` and `Makefile`. The default one is user `sine` with uid `1000` and workspace `/home/sine/code/ssine/robot-demo`.

```bash
make build-docker
make run-docker
make attach-docker
```

## Camera Calibration

Run `sudo python3 ./script/camera/capture_chessboard.py`, press `c` to capture current image. Capture about 15 images and press `q` tu quit.

Run `python3 ./script/camera/calibrate_camera.py` to get camera parameters.

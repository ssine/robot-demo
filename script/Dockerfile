# a VM-like docker image for playing with ros humble
FROM ros:humble

# install dependencies
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    # gst
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    # opencv
    libopencv-dev python3-opencv \
    # ros2 packages
    python3-numpy libboost-python-dev

RUN apt-get install -y ros-humble-rviz2 ros-humble-rqt-graph ros-humble-image-transport-plugins vim

RUN apt-get install -y python3-pip \
    && pip install megapi pyserial

RUN apt-get install -y tmux openssh-client ros-humble-tf-transformations

RUN pip install apriltag transforms3d

RUN sed -i '86 i \ \ \ \ \ \ \ \ th.daemon = True' /usr/local/lib/python3.10/dist-packages/megapi_python3.py

# forward display to workstation
ENV DISPLAY=192.168.2.68:0
ENV LIBGL_ALWAYS_SOFTWARE=1

# set up user environment
ENV UID=1000
ENV GID=1000
RUN groupadd -g ${GID} sine && \
    useradd -rm -d /home/sine \
    -s /bin/bash \
    -u ${UID} -g ${GID} -G root,sudo,video,dialout \
    -p "$(openssl passwd -1 liusiyao)" \
    sine
USER sine
WORKDIR /home/sine

RUN echo "source /opt/ros/humble/setup.bash" >> /home/sine/.bashrc

CMD bash

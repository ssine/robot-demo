FROM althack/ros2:humble-dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        # gst
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
        # opencv
        libopencv-dev python3-opencv \
        # ros2 packages
        python3-numpy libboost-python-dev  \
        && apt-get autoremove -y \
        && apt-get clean -y
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

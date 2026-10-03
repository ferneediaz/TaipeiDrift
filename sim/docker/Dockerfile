# ROS 2 Jazzy + Gazebo Harmonic, with a browser desktop (noVNC) for the Gazebo window.
# ros:jazzy-ros-base is published for arm64 and amd64, so it runs natively on Apple Silicon
# and on x86 laptops. (osrf/ros:jazzy-desktop is amd64 only and would run emulated on a Mac.)
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-rqt-image-view \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-rosbag2-storage-mcap \
        python3-numpy python3-opencv python3-scipy \
        xvfb x11vnc novnc websockify fluxbox supervisor xterm mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# No GPU inside Docker on a Mac: render with Mesa's CPU rasteriser.
# No D-Bus session in the container: without the last three, the Gazebo GUI hangs
# at "Create main window" waiting for D-Bus.
ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    DBUS_SESSION_BUS_ADDRESS=disabled: \
    QT_ACCESSIBILITY=0 \
    NO_AT_BRIDGE=1

COPY supervisord.conf /etc/supervisor/conf.d/desktop.conf
COPY fluxbox-apps /root/.fluxbox/apps
# http://localhost:6080 connects straight away and scales the desktop to the browser window
COPY index.html /usr/share/novnc/index.html
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /ws/TaipeiDrift
EXPOSE 6080
CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]

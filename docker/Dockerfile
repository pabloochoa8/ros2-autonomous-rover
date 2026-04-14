FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Dependencias base + repositorios ROS 2
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list

# ROS 2 Jazzy + Gazebo Harmonic + herramientas de desarrollo
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy \
    ros-jazzy-v4l2-camera \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-opencv \
    git \
    && rm -rf /var/lib/apt/lists/*

# OpenCV contrib (para ArUco)
RUN pip3 install --no-cache-dir opencv-contrib-python

# rosdep
RUN rosdep init && rosdep update

# Soporte X11 para GUI (Gazebo + RViz)
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Variables de entorno para X11 y Gazebo
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=0

# Source ROS 2 automáticamente
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Directorio de trabajo
WORKDIR /root/rover_ws

# Entrypoint: source ROS 2 y ejecutar el comando
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

################################################################
# Dockerfile per l'installazione di ROS 2 Humble su Ubuntu 22.04
################################################################

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Pacchetti base
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    sudo \
    git \
    wget \
    build-essential \
    x11-apps \
    libgl1-mesa-glx \
    libx11-dev \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Aggiunta chiavi e repository ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list

# Installa ROS 2 Desktop completo
RUN apt-get update && apt-get install -y ros-humble-desktop

# Installa strumenti ROS (ora i pacchetti sono disponibili)
RUN apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Inizializza rosdep
RUN rosdep init && rosdep update

# Crea utente non-root
RUN useradd -m -s /bin/bash sebab && echo "sebab ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Setup ROS per l’utente
RUN echo "source /opt/ros/humble/setup.bash" >> /home/sebab/.bashrc && \
    chown sebab:sebab /home/sebab/.bashrc

USER sebab
WORKDIR /home/sebab
RUN mkdir -p /home/sebab/ros2_ws

CMD ["bash"]

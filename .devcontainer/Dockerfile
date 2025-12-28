FROM ghcr.io/sloretz/ros:humble-desktop-full

# 定义用户和用户组
ARG USERNAME=skybase
WORKDIR /home/$USERNAME

RUN apt update && apt install -y \
    python3-pip \
    gcc \
    openssh-server\
    unzip -y \
    curl \
    git \
    wget \
    vim \
    iputils-ping \
    libglpk-dev \
    doxygen doxygen-latex \
    software-properties-common\
    liburdfdom-dev liboctomap-dev libassimp-dev \
    libglfw3-dev freeglut3-dev

RUN  apt-get install ros-humble-joy

RUN apt install -y \
gdb \
gdbserver

RUN apt install -y \
python3-colcon-common-extensions \ 
ros-humble-foxglove-bridge

RUN apt install -y \
    ros-humble-eigenpy\
    ros-humble-pinocchio

USER ${USERNAME}
USER root

# 安装mujoco
RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.3.2/mujoco-3.3.2-linux-x86_64.tar.gz
RUN tar -zxvf mujoco-3.3.2-linux-x86_64.tar.gz
RUN mv mujoco-3.3.2 /opt/mujoco

# # 编译安装 qpOASES
# RUN mkdir -p /home/USERNAME/legged/qpOASES/build && \
#     cd /home/USERNAME/legged/qpOASES/build && \
#     cmake .. && \
#     make install

# 配置环境变量
RUN echo "export LD_LIBRARY_PATH=/opt/mujoco/lib:\$LD_LIBRARY_PATH" >>  ~/.bashrc

# 恢复用户权限
RUN echo "chown -R 1000:1000 /home/skybase/legged" >> ~/.bashrc 
RUN echo "source /opt/ros/humble/setup.bash" >>  ~/.bashrc
RUN echo "source install/setup.bash" >>  ~/.bashrc


CMD ["/bin/bash"]


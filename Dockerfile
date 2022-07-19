FROM ros:noetic

ENV CATKIN_WS=/root/catkin_ws
ENV N_PROC = 2
ENV DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics



RUN apt-get update && apt-get install -y \
    cmake \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \  
    xvfb libfmt-dev \
    ros-noetic-image-geometry \
    ros-noetic-pcl-ros \
    ros-noetic-image-proc \
    ros-noetic-tf-conversions \
    ros-noetic-sophus \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-geometry-msgs && \
    # System dependencies
    apt-get install -y git \
    build-essential unzip pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libvtk6-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    libgl1-mesa-dev libglew-dev \
    python3-wstool python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 

RUN cd /root/ && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git fetch && \
    git checkout tags/v0.6 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j2 && \
    rm -rf Pangolin

RUN cd /var/tmp && \
    git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
    -D SOPHUS_INSTALL=ON .. && \
    make install -j2 && \
    rm -rf Sophus

# Setup catkin workspace
RUN mkdir -p $CATKIN_WS/src/vslam2/ && \
    cd ${CATKIN_WS} && \
    catkin init && \
    catkin config \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release && \
    catkin config --merge-devel

COPY ./ ${CATKIN_WS}/src/vslam2

# RUN cd $CATKIN_WS/src && \ 
#     git clone https://github.com/hellovuong/vslam.git

RUN rosdep update

# RUN source /opt/ros/noetic/setup.bash && \
RUN cd $CATKIN_WS/src && \
    wstool init && \
    wstool merge vslam2/vslam2.rosinstall   && \
    wstool update

RUN cd $CATKIN_WS && \
    catkin build -j2 catkin_simple && \ 
    catkin build -j2 dbow2_catkin && \ 
    catkin build -j2 g2o_catkin 

RUN cd $CATKIN_WS/src/vslam2/ORB_SLAM3  && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j2

RUN cd $CATKIN_WS && \
    catkin build -j2


WORKDIR ${CATKIN_WS}
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8   
RUN cd ${CATKIN_WS} && \ 
    sed -i '/exec "$@"/i \
    source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
EXPOSE 11311
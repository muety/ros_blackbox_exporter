# To use custom message definitions, mount them to /home/ros_ws/custom

FROM ros:noetic

RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/ros_ws/src
WORKDIR /home/ros_ws

COPY . /home/ros_ws/src/ros_blackbox_exporter

RUN pip install -r /home/ros_ws/src/ros_blackbox_exporter/requirements.txt

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash && source /home/ros_ws/devel/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "bash /home/ros_ws/src/ros_blackbox_exporter/docker/entrypoint.sh && source /opt/ros/noetic/setup.bash && source /home/ros_ws/devel/setup.bash && roslaunch ros_blackbox_exporter ros_blackbox_exporter.launch"]

ENV ROS_MASTER_URI=http://172.17.0.1:11311
ENV ROS_IP=172.17.0.1
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

# To use custom message definitions, mount them to /home/ros_ws/custom

COPY docker/entrypoint.sh /home/ros_ws/entrypoint.sh
RUN chmod +x /home/ros_ws/entrypoint.sh

CMD ["/bin/bash", "-c", "/home/ros_ws/entrypoint.sh && roslaunch ros_blackbox_exporter ros_blackbox_exporter.launch"]

ENV ROS_MASTER_URI=http://172.17.0.1:11311
ENV ROS_IP=172.17.0.1
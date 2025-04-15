# To use custom message definitions, mount them to /home/ros_ws/custom

FROM ros:jazzy

RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/ros_ws/src
WORKDIR /home/ros_ws

COPY . /home/ros_ws/src/ros_blackbox_exporter

RUN pip install --break-system-packages -r /home/ros_ws/src/ros_blackbox_exporter/requirements.txt

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash && source /home/ros_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "bash /home/ros_ws/src/ros_blackbox_exporter/docker/entrypoint.sh && \
    source /opt/ros/jazzy/setup.bash && \
    source /home/ros_ws/install/setup.bash && \
    ros2 launch ros_blackbox_exporter ros_blackbox_exporter.launch.py ${LAUNCH_ARGS}"]

ENV ROS_DOMAIN_ID=0
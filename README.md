# ROS Blackbox Exporter

A ROS 1 node to export basic topic statistics for [Prometheus](https://prometheus.io).

## Requirements
* ROS Noetic
* Python >= 3.8

## Usage
### Installing dependencies
```bash
pip install -r requirements.txt
```

### With ROS
```bash
roslaunch ros_blackbox_exporter ros_blackbox_exporter.launch

# Optionally use external config file:
# roslaunch ros_blackbox_exporter ros_blackbox_exporter.launch config_file:="my_config.yml"
``` 

### Standalone
```bash
cd src
python3 ros_blackbox_exporter.py
```

### In Docker
```bash
# Replace IP with the actual IP of your ROS master

docker build . -t ros_blackbox_exporter
docker run -d \
    -v $(pwd)/exporter.yml:/home/ros_ws/src/ros_blackbox_exporter/exporter.yml \
    -e ROS_MASTER_URI=http://172.17.0.1:11311 \
    -e ROS_IP=172.17.0.1 \
    -p 8866:8866 \
    ros_blackbox_exporter
```

To include **custom message definitions**, mount a directory containing all your packages into the container at `/home/ros_ws/custom`.

## Metrics
Metrics are exposed at http://localhost:8866.

```
# HELP ros_blackbox_topic_ok Whether topic is available or not
# TYPE ros_blackbox_topic_ok gauge
ros_blackbox_topic_ok{topic="/sensors/camera/front_medium/image_rect_color/compressed"} 1.0

# HELP ros_blackbox_topic_offset Time since last message on this topic in seconds
# TYPE ros_blackbox_topic_offset gauge
ros_blackbox_topic_offset{topic="/sensors/camera/front_medium/image_rect_color/compressed",type="sensor_msgs/CompressedImage"} 1.4e-05

# HELP ros_blackbox_topic_delay Delay of messages published on this topic in seconds
# TYPE ros_blackbox_topic_delay gauge
ros_blackbox_topic_delay{topic="/sensors/camera/front_medium/image_rect_color/compressed",type="sensor_msgs/CompressedImage"} 76459.189029

# HELP ros_blackbox_topic_rate Current publishing rate on the topic
# TYPE ros_blackbox_topic_rate gauge
ros_blackbox_topic_rate{topic="/sensors/camera/front_medium/image_rect_color/compressed",type="sensor_msgs/CompressedImage"} 20.01

# HELP ros_blackbox_topic_bw Current bandwidth published on the topic
# TYPE ros_blackbox_topic_bw gauge
ros_blackbox_topic_bw{topic="/sensors/camera/front_medium/image_rect_color/compressed",type="sensor_msgs/CompressedImage"} 396525.04
```

### Offset vs. delay
Delay is the time difference between the message's header timestamp and the current time. Offset is the interval between when the message was received by the node and the current time.

## To Do
* Improved error handling (e.g. auto-reconnect, etc.)
* "On-demand" subscriptions, inspired by [blackbox_exporter](https://github.com/prometheus/blackbox_exporter)'s `?target=` syntax
* ROS-free rewrite, e.g. using [goroslib](https://github.com/bluenviron/goroslib)

## License
MIT
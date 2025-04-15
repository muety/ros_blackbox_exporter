# ROS Blackbox Exporter

A ROS 2 node to export basic topic statistics for [Prometheus](https://prometheus.io).

## Requirements
* ROS 2 Jazzy
* Python >= 3.11
* python3-venv

## Setup
### Clone repo
Clone the package into your ROS2 workspace's `src/` folder.

```bash
git clone https://github.com/muety/ros_blackbox_exporter.git
```

### Install ROS dependencies

```bash
rosdep install -i --from-path src
```

### Install Python depenencies
```bash
cd src/ros_blackbox_exporter
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cd ../..
```

### Build workspace
```bash
colcon build --packages-select ros_blackbox_exporter
```

## Usage

### With ROS
```bash
ros2 launch \
    ros_blackbox_exporter \
    ros_blackbox_exporter.launch.py  \
    config_file:=/some/folder/exporter.yml
``` 

### In Docker
```bash
docker build . -t ros_blackbox_exporter
docker run -d \
    -v $(pwd)/exporter.yml:/opt/exporter.yml \
    -e LAUNCH_ARGS="config_file:=/opt/exporter.yml" \
    -p 8866:8866 \
    --network host \
    ros_blackbox_exporter
```

To include **custom message definitions**, mount a directory containing all your packages into the container at `/home/ros_ws/custom`.

If your other nodes are running in Docker as well, you'd probably rather want to use isolated networking among them instead of `host` network. Alternatively, there is [discovery server](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html) as well.

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
* Improved error handling / robustness
* "On-demand" subscriptions, inspired by [blackbox_exporter](https://github.com/prometheus/blackbox_exporter)'s `?target=` syntax
* ROS-free rewrite, e.g. using [goroslib](https://github.com/bluenviron/goroslib)

## Developer notes
To get auto-completion for ROS 2 Python libs in VSCode, add the following to your `.vscode/settings.json`.

```json
{
   "python.autoComplete.extraPaths": ["/opt/ros/jazzy/lib/python3.12/site-packages"],
   "python.analysis.extraPaths": ["/opt/ros/jazzy/lib/python3.12/site-packages"]
}
```

## License
MIT
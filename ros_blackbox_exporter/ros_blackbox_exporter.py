#!/usr/bin/python3

import argparse
from dataclasses import dataclass, field
from functools import lru_cache
import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Type, Union
from prometheus_client import REGISTRY, CollectorRegistry, start_http_server, Gauge
from prometheus_client.registry import Collector
import yaml

import rclpy
from rclpy.subscription import Subscription
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from ros2topic.verb.bw import ROSTopicBandwidth as ROS2TopicBandwidth
from ros2topic.verb.hz import ROSTopicHz as ROS2TopicHz
from ros2topic.verb.delay import ROSTopicDelay as ROS2TopicDelay
from rosidl_runtime_py.utilities import get_message

logging.basicConfig(level=logging.INFO)

metric_topic_ok = Gauge('ros_blackbox_topic_ok', 'Whether topic is available or not', ['topic'])
metric_topic_delay = Gauge('ros_blackbox_topic_delay', 'Delay of messages published on this topic (seconds)', ['topic', 'type'])
metric_topic_rate = Gauge('ros_blackbox_topic_rate', 'Current publishing rate on the topic (Hz)', ['topic', 'type'])
metric_topic_bandwidth = Gauge('ros_blackbox_topic_bw', 'Current bandwidth published on the topic (bytes / s)', ['topic', 'type'])
metric_topic_offset = Gauge('ros_blackbox_topic_offset', 'Time since last message on this topic (seconds)', ['topic', 'type'], registry=CollectorRegistry())

subscriptions: Dict[str, 'ROSSubscription'] = {}

@dataclass
class MetricsConfig:
    offset: bool = True
    delay: bool = False
    bw: bool = True
    rate: bool = True

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> 'MetricsConfig':
        return cls(**payload)

@dataclass
class TopicConfig:
    name: str
    metrics: MetricsConfig

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> 'TopicConfig':
        metrics = MetricsConfig.from_dict(payload.pop('metrics'))
        return cls(metrics=metrics, **payload)

@dataclass
class Config:
    addr: str = '127.0.0.1'
    port: int = 8866
    topics: List[TopicConfig] = field(default_factory=list)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> 'Config':
        topics = [ TopicConfig.from_dict(t) for t in payload.pop('topics') ]
        return cls(topics=topics, **payload)
    
class ROSTopicOffset:
    def __init__(self, node: Node):
        self.lock: threading.Lock = threading.Lock()
        self.ts: float = 0.
    
    def callback_offset(self, data):
        with self.lock:
            self.ts = time.time()
    
    def get_offset(self) -> float:
        return time.time() - self.ts if self.ts > 0 else -1
    
class ROSTopicBandwidth(ROS2TopicBandwidth):
    def get_bw(self):
        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = self.clock.now()
            t0 = self.times[0]
            
            total = sum(self.sizes)
            bytes_per_s = total / ((tn - t0).nanoseconds * 1e-9)
            mean = total / n
            return mean

@lru_cache(maxsize=None)  # singleton
class TopicOffsetCollector(Collector):
    def __init__(self, metric: Gauge):
        self.metric: Gauge = metric
        self.offsets: List[ROSTopicOffset] = []
        self.labels: List[Union[Dict[str, str], Callable[[], Dict[str, str]]]] = []
        super().__init__()
        REGISTRY.register(self)

    def add(self, offset: ROSTopicOffset, labels: Union[Dict[str, str], Callable[[], Dict[str, str]]] = {}):
        self.offsets.append(offset)
        self.labels.append(labels)

    def collect(self):
        for i in range(len(self.offsets)):
            labels: Dict[str, str] = self.labels[i] if isinstance(self.labels[i], dict) else self.labels[i]()
            self.metric.labels(**labels).set(round(self.offsets[i].get_offset(), 6))
        yield from self.metric.collect()


class ROSSubscription:
    def __init__(self, node: Node, topic: str, offset: bool = True, delay: bool = True, bw: bool = False, rate: bool = True):
        self.node = node
        
        self.topic: str = topic
        self.msgtype: str = None
        self.msgclass: Type = None

        self.hz: Optional[ROS2TopicHz] = ROS2TopicHz(self.node, 1000) if rate else None
        self.delay: Optional[ROS2TopicDelay] = ROS2TopicDelay(self.node, 1000) if delay else None
        self.bw: Optional[ROSTopicBandwidth] = ROSTopicBandwidth(self.node, 100) if bw else None
        self.offset: Optional[ROSTopicOffset] = ROSTopicOffset(self.node) if offset else None

        self.offset_collector: TopicOffsetCollector = TopicOffsetCollector(metric_topic_offset)  # get global singleton instance
        self.offset_collector.add(self.offset, self.get_labels)

        self.sub: Subscription = None
        
    def listen(self) -> 'ROSSubscription':
        def try_subscribe():
            while True:
                try:
                    self.msgtype = self.get_topic_type(self.topic)
                    self.msgclass = get_message(self.msgtype)
                    self.sub = self.node.create_subscription(self.msgclass, self.topic, self.callback, qos_profile_sensor_data, raw=True)
                    self.node.get_logger().info(f'Subscription to {self.topic} successful')
                    return
                except (AttributeError, KeyError) as e:
                    print(e)
                    self.node.get_logger().warn(f'Topic {self.topic} not yet available, waiting ...')
                    time.sleep(5)

        threading.Thread(target=try_subscribe, daemon=True).start()

    def get_labels(self) -> Dict[str, str]:
        return dict(topic=self.topic, type=self.msgtype)

    def get_topic_type(self, topic: str) -> str:
        for tname, ttypes in self.node.get_topic_names_and_types():
            if tname == topic:
                return ttypes[0]
        raise KeyError(f'topic {topic} not published')

    def callback(self, raw):
        global metric_topic_offset, metric_topic_delay, metric_topic_rate, metric_topic_bandwidth

        if self.hz:
            self.hz.callback_hz(raw)
            if (hz := self._get_hz()) >= 0:
                metric_topic_rate.labels(topic=self.topic, type=self.msgtype).set(hz)
        
        if self.delay:
            if raw is None:
                metric_topic_ok.labels(topic=self.topic).set(0)
            else:
                msg = deserialize_message(raw, self.msgclass)
                self.delay.callback_delay(msg)
                metric_topic_delay.labels(topic=self.topic, type=self.msgtype).set(self._get_delay())

        if self.bw:
            self.bw.callback(raw)
            metric_topic_bandwidth.labels(topic=self.topic, type=self.msgtype).set(self._get_bw())

        if self.offset:
            self.offset.callback_offset(raw)


    def _get_delay(self) -> float:
        delay = self.delay.get_delay()
        return round(delay[0] if delay else -1., 6)
    
    def _get_offset(self) -> float:
        return round(self.offset.get_offset(), 6)
    
    def _get_bw(self) -> float:
        bw = self.bw.get_bw()
        return round(bw if bw is not None else -1., 2)
    
    def _get_hz(self) -> float:
        hz = self.hz.get_hz()
        return round(hz[0] * 1e9 if hz else -1. , 2)

class ROSBlackboxExporterNode(Node):
    def __init__(self, topics: List[TopicConfig]):
        node_name: str = 'ros_blackbox_exporter'
        
        super().__init__(node_name)
        
        for t in topics:
            self.subscribe(t.name, t.metrics)

    def on_subscribe_success(self, topic: str):
        metric_topic_ok.labels(topic=topic).set(1)

    def on_subscribe_failed(self, topic: str):
        metric_topic_ok.labels(topic=topic).set(0)

    def subscribe(self, topic: str, metrics: MetricsConfig):
        try:
            self.get_logger().info(f'Subscribing to {topic}')
            sub: ROSSubscription = ROSSubscription(self, topic, **metrics.__dict__).listen()
            subscriptions[topic] = sub
            self.on_subscribe_success(topic)
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to {topic}, {e}')
            self.on_subscribe_failed(topic)


def run_ros(topics: List[TopicConfig], args=None):
    rclpy.init(args=args)
    
    node = ROSBlackboxExporterNode(topics)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

def read_config(path: str):
    with open(path, 'r') as f:
        config_dict = yaml.safe_load(f)
    return Config.from_dict(config_dict)

def main(args=None):
    parser = argparse.ArgumentParser('ROS Blackbox Exporter')
    parser.add_argument('--config', '-c', default='../exporter.yml', type=str, help='Path to config YAML')
    parsed_args, _ = parser.parse_known_args()

    config = read_config(parsed_args.config)

    server, thread = start_http_server(port=config.port, addr=config.addr)
    
    run_ros(config.topics, None)
    
    server.shutdown()
    thread.join()
    
if __name__ == '__main__':
    main()
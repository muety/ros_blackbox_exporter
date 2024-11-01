#!/usr/bin/python3

import argparse
from dataclasses import dataclass, field
import logging
import threading
import time
from typing import Any, Dict, List, Optional, Type
from prometheus_client import start_http_server, Gauge
import yaml
import rospy, rostopic

logging.basicConfig(level=logging.INFO)

metric_topic_offset = Gauge('ros_blackbox_topic_offset', 'Time since last message on this topic (seconds)', ['topic', 'type'])
metric_topic_delay = Gauge('ros_blackbox_topic_delay', 'Delay of messages published on this topic (seconds)', ['topic', 'type'])
metric_topic_rate = Gauge('ros_blackbox_topic_rate', 'Current publishing rate on the topic (Hz)', ['topic', 'type'])
metric_topic_bandwidth = Gauge('ros_blackbox_topic_bw', 'Current bandwidth published on the topic (bytes / s)', ['topic', 'type'])

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
    def __init__(self):
        self.lock: threading.Lock = threading.Lock()
        self.ts: float = 0.
    
    def callback_offset(self, data):
        with self.lock:
            self.ts = time.time()
    
    def get_offset(self) -> float:
        return time.time() - self.ts
    
class ROSTopicBandwidth(rostopic.ROSTopicBandwidth):
    def get_bw(self):
        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = time.time()
            t0 = self.times[0]
            
            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n
            return mean

class ROSSubscription:
    def __init__(self, topic: str, offset: bool = True, delay: bool = True, bw: bool = False, rate: bool = True):
        self.topic: str = topic
        self.msgtype: str = None
        self.msgclass: Type = None

        self.hz: Optional[rostopic.ROSTopicHz] = rostopic.ROSTopicHz(1000) if rate else None
        self.delay: Optional[rostopic.ROSTopicDelay] = rostopic.ROSTopicDelay(1000) if delay else None
        self.bw: Optional[ROSTopicBandwidth] = ROSTopicBandwidth(100) if bw else None
        self.offset: Optional[ROSTopicOffset] = ROSTopicOffset() if offset else None

        self.sub: rospy.Subscriber = None

    def listen(self) -> 'ROSSubscription':
        topic_type, _, _ = rostopic.get_topic_class(self.topic)
        self.msgtype = topic_type._type
        self.msgclass = topic_type
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.callback)


    def callback(self, raw: rospy.AnyMsg):
        global metric_topic_offset, metric_topic_delay, metric_topic_rate, metric_topic_bandwidth

        msg = self.msgclass().deserialize(raw._buff)

        if self.hz:
            self.hz.callback_hz(raw)
            metric_topic_rate.labels(topic=self.topic, type=self.msgtype).set(self._get_hz())

        if self.delay:
            self.delay.callback_delay(msg)
            metric_topic_delay.labels(topic=self.topic, type=self.msgtype).set(self._get_delay())

        if self.bw:
            self.bw.callback(raw)
            metric_topic_bandwidth.labels(topic=self.topic, type=self.msgtype).set(self._get_bw())

        if self.offset:
            self.offset.callback_offset(raw)
            metric_topic_offset.labels(topic=self.topic, type=self.msgtype).set(self._get_offset())


    def _get_offset(self) -> float:
        return round(self.offset.get_offset(), 6)
    
    def _get_delay(self) -> float:
        delay = self.delay.get_delay()
        return round(delay[0] if delay else -1., 6)
    
    def _get_bw(self) -> float:
        bw = self.bw.get_bw()
        return round(bw if bw is not None else -1., 2)
    
    def _get_hz(self) -> float:
        hz = self.hz.get_hz()
        return round(hz[0] if hz else -1. , 2)

def subscribe(topic: str, metrics: MetricsConfig):
    try:
        sub: ROSSubscription = ROSSubscription(topic, **metrics.__dict__).listen()
        subscriptions[topic] = sub
        rospy.loginfo(f'Subscribed to {topic}')
    except Exception as e:
        rospy.logerr('Failed to subscribe to %s (%s)', topic, e)


def run_ros(topics: List[TopicConfig]):
    rospy.init_node('ros_blackbox_exporter')

    for t in topics:
        subscribe(t.name, t.metrics)
    
    rospy.spin()

def read_config(path: str):
    with open(path, 'r') as f:
        config_dict = yaml.safe_load(f)
    return Config.from_dict(config_dict)

if __name__ == '__main__':
    parser = argparse.ArgumentParser('ROS Blackbox Exporter')
    parser.add_argument('--config', '-c', default='../exporter.yml', type=str, help='Path to config YAML')
    parsed_args, _ = parser.parse_known_args()

    config = read_config(parsed_args.config)

    server, thread = start_http_server(port=config.port, addr=config.addr)
    
    run_ros(config.topics)
    
    server.shutdown()
    thread.join()
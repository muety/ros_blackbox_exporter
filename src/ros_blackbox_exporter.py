#!/usr/bin/python3

import argparse
from dataclasses import dataclass, field
from functools import lru_cache
import logging
import threading
import time
from typing import Any, Dict, List, Optional, Type
from prometheus_client import REGISTRY, CollectorRegistry, start_http_server, Gauge
from prometheus_client.registry import Collector
import yaml
import rosgraph
import rospy, rostopic

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
    def __init__(self):
        self.lock: threading.Lock = threading.Lock()
        self.ts: float = 0.
    
    def callback_offset(self, data):
        with self.lock:
            self.ts = time.time()
    
    def get_offset(self) -> float:
        return time.time() - self.ts if self.ts > 0 else -1
    
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

@lru_cache(maxsize=None)  # singleton
class TopicOffsetCollector(Collector):
    def __init__(self, metric: Gauge):
        self.metric: Gauge = metric
        self.offsets: List[ROSTopicOffset] = []
        self.labels: List[Dict[str, str]] = []
        super().__init__()
        REGISTRY.register(self)

    def add(self, offset: ROSTopicOffset, labels: Dict[str, str] = {}):
        self.offsets.append(offset)
        self.labels.append(labels)

    def collect(self):
        for i in range(len(self.offsets)):
            self.metric.labels(**self.labels[i]).set(round(self.offsets[i].get_offset(), 6))
        yield from self.metric.collect()


class ROSSubscription:
    def __init__(self, topic: str, offset: bool = True, delay: bool = True, bw: bool = False, rate: bool = True, ):
        self.topic: str = topic
        self.msgtype: str = None
        self.msgclass: Type = None

        self.hz: Optional[rostopic.ROSTopicHz] = rostopic.ROSTopicHz(1000) if rate else None
        self.delay: Optional[rostopic.ROSTopicDelay] = rostopic.ROSTopicDelay(1000) if delay else None
        self.bw: Optional[ROSTopicBandwidth] = ROSTopicBandwidth(100) if bw else None
        self.offset: Optional[ROSTopicOffset] = ROSTopicOffset() if offset else None

        self.offset_collector: TopicOffsetCollector = TopicOffsetCollector(metric_topic_offset)  # get global singleton instance
        self.offset_collector.add(self.offset, dict(topic=self.topic, type=self.msgtype))

        self.sub: rospy.Subscriber = None

    def listen(self) -> 'ROSSubscription':
        def try_subscribe():
            while True:
                try:
                    topic_type, _, _ = rostopic.get_topic_class(self.topic)
                    self.msgtype = topic_type._type
                    self.msgclass = topic_type
                    self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.callback, queue_size=1)
                    rospy.loginfo(f'Subscription to {self.topic} successful')
                    return
                except AttributeError:
                    rospy.logwarn(f'Topic {self.topic} not yet available, waiting ...')
                    time.sleep(5)

        threading.Thread(target=try_subscribe, daemon=True).start()


    def callback(self, raw: rospy.AnyMsg):
        global metric_topic_offset, metric_topic_delay, metric_topic_rate, metric_topic_bandwidth

        msg = self.msgclass().deserialize(raw._buff)
        
        if msg is None:
            metric_topic_ok.labels(topic=self.topic).set(0)

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
        return round(hz[0] if hz else -1. , 2)

def on_subscribe_success(topic: str):
    metric_topic_ok.labels(topic=topic).set(1)

def on_subscribe_failed(topic: str):
    metric_topic_ok.labels(topic=topic).set(0)

def subscribe(topic: str, metrics: MetricsConfig):
    try:
        rospy.loginfo(f'Subscribing to {topic}')
        sub: ROSSubscription = ROSSubscription(topic, **metrics.__dict__).listen()
        subscriptions[topic] = sub
        on_subscribe_success(topic)
    except Exception as e:
        rospy.logerr('Failed to subscribe to %s (%s)', topic, e)
        on_subscribe_failed(topic)

def watchdog(node_name: str):
    master = rosgraph.Master(node_name)
    while True:
        try:
            master.getSystemState()
            time.sleep(5)
        except ConnectionRefusedError:
            rospy.logwarn('Lost connection to master, exiting now')
            rospy.signal_shutdown('master unavailable')  # TODO: exit with error status code
            return  # TODO: try reconnect instead of crashing (see #2)


def run_ros(topics: List[TopicConfig]):
    node_name: str = 'ros_blackbox_exporter'
    rospy.init_node(node_name)

    watchdog_thread = threading.Thread(target=watchdog, args=(node_name,), daemon=True)
    watchdog_thread.start()

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
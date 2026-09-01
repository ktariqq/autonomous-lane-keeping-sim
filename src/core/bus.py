"""Minimal synchronous pub/sub bus standing in for ROS 2 topics. There is
no threading/executor — the main loop is the single "spin" — so this is
a synchronous analogue of a ROS 2 graph rather than a real DDS transport,
but node boundaries and topic names match what an `rqt_graph` would show."""


class Bus:
    def __init__(self):
        self._topics = {}

    def publish(self, topic, msg):
        self._topics[topic] = msg

    def get(self, topic, default=None):
        return self._topics.get(topic, default)
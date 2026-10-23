"""Small ROS1/ROS2 adapter; ROS modules are imported only in their own runtime."""
import os
import threading
import time


def _modern_find_caller(self, stack_info=False, stacklevel=1):
    """Use Python's bounded frame walker with Noetic on Python >=3.11.

    Old rosgraph RospyLogger.findCaller searches for an exact line returned
    by logging, but can loop at the outermost frame on modern Python.
    Delegating to logging avoids that loop without patching system ROS files.
    """
    import logging
    return logging.Logger.findCaller(self, stack_info=stack_info, stacklevel=stacklevel + 1)


class RosRuntime:
    def __init__(self, name):
        self.version = int(os.environ.get('ROS_VERSION', '2'))
        self.name = name
        self.lock = threading.Lock()
        self.ready = None
        self.received = None
        self.refs = []
        if self.version == 1:
            import rospy
            import sys
            if sys.version_info >= (3, 11):
                from rosgraph.roslogging import RospyLogger
                RospyLogger.findCaller = _modern_find_caller
            self.ros = rospy
            rospy.init_node(name, disable_signals=True)
            self.node = None
        elif self.version == 2:
            import rclpy
            from rclpy.node import Node
            self.ros = rclpy
            rclpy.init()
            self.node = Node(name)
        else:
            raise ValueError('ROS_VERSION must be 1 or 2')

    def param(self, name, default):
        if self.version == 1:
            return self.ros.get_param('~' + name.replace('.', '/'), default)
        self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def log(self, text, error=False):
        if self.version == 1:
            (self.ros.logwarn if error else self.ros.loginfo)(text)
        else:
            logger = self.node.get_logger()
            (logger.warning if error else logger.info)(text)

    def publisher(self, topic):
        from std_msgs.msg import String
        if self.version == 1:
            publisher = self.ros.Publisher(topic.replace('~/', '~'), String, queue_size=10)
        else:
            publisher = self.node.create_publisher(String, topic, 10)
        self.refs.append(publisher)
        return lambda text: publisher.publish(String(data=text))

    def subscribe_text(self, topic, callback):
        from std_msgs.msg import String
        if self.version == 1:
            subscription = self.ros.Subscriber(topic.replace('~/', '~'), String,
                                               lambda msg: callback(msg.data), queue_size=1)
        else:
            subscription = self.node.create_subscription(String, topic, lambda msg: callback(msg.data), 10)
        self.refs.append(subscription)

    def trigger_service(self, topic, callback):
        from std_srvs.srv import Trigger
        if self.version == 1:
            from std_srvs.srv import TriggerResponse

            def handle(_request):
                ok, message = callback()
                return TriggerResponse(success=ok, message=message)

            service = self.ros.Service(topic.replace('~/', '~'), Trigger, handle)
        else:
            def handle(_request, response):
                response.success, response.message = callback()
                return response

            service = self.node.create_service(Trigger, topic, handle)
        self.refs.append(service)

    def record_service(self, topic, callback):
        """Backward-compatible name for a Trigger service."""
        return self.trigger_service(topic, callback)

    def watch_perception(self, topic):
        from std_msgs.msg import Bool
        self.ready_topic = topic

        def update(message):
            with self.lock:
                self.ready = message.data
                self.received = time.monotonic()

        if self.version == 1:
            subscription = self.ros.Subscriber(topic, Bool, update, queue_size=1)
        else:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            # VOLATILE accepts both volatile and transient-local publishers;
            # BEST_EFFORT also accepts common sensor publisher QoS profiles.
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            subscription = self.node.create_subscription(Bool, topic, update, qos)
        self.refs.append(subscription)

    def perception_status(self):
        with self.lock:
            return {'ros_version': self.version, 'topic': self.ready_topic, 'ready': self.ready,
                    'seconds_since_message': None if self.received is None else
                    round(time.monotonic() - self.received, 3)}

    def list_topics(self):
        if self.version == 1:
            return [{'name': name, 'types': [kind]} for name, kind in self.ros.get_published_topics()]
        return [{'name': name, 'types': kinds} for name, kinds in self.node.get_topic_names_and_types()]

    def ok(self):
        return not self.ros.is_shutdown() if self.version == 1 else self.ros.ok()

    def spin(self):
        if self.version == 1:
            self.ros.spin()
        else:
            from rclpy.executors import ExternalShutdownException
            try:
                self.ros.spin(self.node)
            except ExternalShutdownException:
                pass

    def close(self):
        if self.version == 1:
            if not self.ros.is_shutdown():
                self.ros.signal_shutdown('AI node stopped')
        else:
            if self.ros.ok():
                self.ros.shutdown()
            self.node.destroy_node()

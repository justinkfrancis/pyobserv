from threading import Thread
import time
import random
from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.guard_condition import GuardCondition
from std_msgs.msg import Float64
from rclpy.qos import qos_profile_sensor_data

from pyobserv import Observable, Observer


class Sensor(Observable):
    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        while self.running:
            self.trigger_observers("sensor", random.random())
            time.sleep(0.1)


class ObserverNode(Node, Observer):
    def __init__(self):
        Node.__init__(self, "SensorObserver")
        Observer.__init__(self)

        self.pub = self.create_publisher(Float64, 'sensor/data',
                                         qos_profile_sensor_data)
        self.queue = Queue()
        self.guard = self.crate_guard_condition(self.publish_data)

    # Runs in sensor thread
    def ob_callback(self, value):
        self.queue.put(value)
        self.guard.trigger()

    # Runs in main / ROS thread
    def publish_data(self):
        msg = Float64()
        msg.data = self.queue.get_nowait()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Create Observer/Observable
    sensor = Sensor()
    node = ObserverNode()
    sensor.add_observer(node)
    node.register_event_and_cb("sensor", node.ob_callback)
    t = Thread(target=sensor.run)

    try:
        t.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

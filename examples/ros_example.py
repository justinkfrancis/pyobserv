"""Example ROS node integrating observ library"""

from threading import Thread, get_ident
import time
from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32

from pyobserv import Observable, Observer


class Sensor(Observable):  # pylint: disable=too-few-public-methods
    """Mock Sensor class that inherits Observable class"""

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        """Publish some mock data"""
        for i in range(6):
            self.trigger_observers("sensor", i)
            time.sleep(0.3)


class ObserverNode(Node, Observer):
    """ROS Node class that inherits Observer"""

    def __init__(self):
        Node.__init__(self, "SensorObserver")
        Observer.__init__(self)

        self.pub = self.create_publisher(
            Int32, "sensor/data", qos_profile_sensor_data
        )
        self.queue = Queue()
        self.guard = self.create_guard_condition(self.publish_data)
        self.done = False
        print(f"Main RCLPY thread has id: {get_ident()}")

    def ob_callback(self, value):
        """Callback triggered by oberservable, runs in sensor thread"""
        print(f"Observable published {value} with id: {get_ident()}")
        self.queue.put(value)
        self.guard.trigger()

    def publish_data(self):
        """Function triggered by observable cb, runs in main thread"""
        msg = Int32()
        msg.data = self.queue.get_nowait()
        print(
            f"RCLPY got and published value {msg.data} with id: {get_ident()}"
        )
        self.pub.publish(msg)
        if msg.data == 5:
            self.done = True


def main(args=None):
    """Create mock sensor and publish data in ROS Node"""
    rclpy.init(args=args)

    # Create Observer/Observable
    sensor = Sensor()
    node = ObserverNode()
    sensor.add_observer(node)
    node.register_event_and_cb("sensor", node.ob_callback)
    t = Thread(target=sensor.run)

    try:
        t.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print("Destroying Node and Shutting Down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

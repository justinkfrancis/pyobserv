# PyObserv [er/able]

![CI](https://github.com/justinKfrancis/pyobserv/actions/workflows/python-package.yml/badge.svg)  ![CI](https://github.com/justinKfrancis/pyobserv/actions/workflows/pylint.yml/badge.svg)  [![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)  


PyObserv is a simple python library for implementing the Observerable 
design pattern to trigger one or more callback functions for an event.  One 
difference to other common implementations is that it separates the 
`Observable` class which manages list of observers and notifying them of 
events and the `Observer` class which registers callback functions to 
events. One core use for this library is to integrate sensors with ROS2 
ensuring that all data from a sensor is published, rather than using a 
timer to get and publish sensor data (see ROS2 executor friendly 
implementation below).

```python
from pyobserv import Observer, Observable

observer = Observer()
observable = Observable()

@observer.register_event_and_cb("event")
def foo(msg):
    print(f"I saw {msg}")

observable.add_observer(observer)
observable.trigger_observers("event", data=1)
```

## How to Install

Use pip to install (ideally in a virtual environment)

```bash
pip install pyobserv
```


# How to use

Create a Observer and Observable class:

```python
from pyobserv import Observer, Observable

observer = Observer()
observable = Observable()
```


## Usage of observer.Observer

### `register_event_and_cb`: 

You can register the callback function for a given event in one of two ways:

#### Decorator:

```python
@observer.register_event_and_cb("test")
def multiplier(data):
    print(f"{data*2}")
```

#### Direct Call

```python
def divider(data):
    print(f"{data/2}")
observer.register_event_and_cb("test", divider)
```

### `get_all_events`

To get all events for an Observable:

```python
>>> observer.get_all_events()
['test']
```

### `get_all_callbacks`

To get all callback functions associated with an event:

```python
>>> observer.get_all_callbacks("test")
[<function multiplier at 0x7f6be3cec540>, <function divider at 0x7f6be3cec2c0>]
```

### `get_all_events_and_callbacks`

To get a dictionary of events and their associated callbacks:

```python
>>> observer.get_all_events_and_callbacks()
{'test': [<function multiplier at 0x7f6be3cec540>, <function divider at 
0x7f6be3cec2c0>]}

```

### `unregister_event_or_cb`

You can unregister callbacks in three different ways:

#### Unregister single callback

To unregister a single callback pass the event and function name:

```python
>>> observer.unregister_event_or_cb('test', divider)
>>> observer.get_all_events_and_callbacks()
{'test': [<function multiplier at 0x7f52b2894360>]}
```

#### Unregister all callbacks for an event

To unregister all callbacks for an event pass the event name:

```python
>>> observer.unregister_event_or_cb('test')
>>> observer.get_all_events_and_callbacks()
{}
>>> 
```

#### Unregister all events and callbacks

To unregister all callbacks for all events pass no arguments:

```python
observer.unregister_event_or_cb(event='')
```

## Usage of observable.Observable

### `add_observer`

To add an observer to an observable class:

```python
observable.add_observer(observer)
```

### `trigger_observers`

To call all callbacks related to an event for all observers:

```python
observable.trigger_observers("test", data=123)
```

### `remove_observer`

To remove an observer from an observable:

```python
observable.remove_observer(observer)
```


## ROS2 Executor Friendly Example

One use this library was designed for was to ensure all data from a sensor 
gets published via ROS2.  Other methods involve using a timer to acquire 
data from a sensor but this has the issue of having to deal with repeat or
lost data.  It is important that you use a guard condition to trigger 
anything done by the executor (such as publish) as it is proper ROS2
convention to not use ROS functions outside the executor thread.

```python
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
        self.guard = self.create_guard_condition(self.publish_data)

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
```

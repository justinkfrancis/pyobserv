"""Basic"""

from pyobserv import Observable, Observer


class Sensor(Observable): # pylint: disable=too-few-public-methods
    """Mock Sensor class that inherits Observable class"""

    def __init__(self):
        super().__init__()
        self.value = 1
        self.other_value = 2


class MyObserver(Observer):
    """Class that with observe observable class and call a function on event"""

    def __init__(self):
        super().__init__()
        self.history = []

    def callback(self, data):
        """Prints data and adds it to the history"""
        self.history.append(data)
        print(f"I saw {data}")

    def callback2(self, data):
        """Also prints data and adds it to the history"""
        self.history.append(data)
        print(f"I also saw {data}")


def main():
    """Create inherited Observer and Observable and trigger events"""
    observable = Sensor()
    my_observer = MyObserver()

    my_observer.register_event_and_cb(
        "event1", my_observer.callback, my_observer.callback2
    )
    my_observer.register_event_and_cb("event2", my_observer.callback)

    observable.add_observer(my_observer)
    observable.trigger_observers("event1", observable.value)
    observable.trigger_observers("event2", observable.other_value)
    print(f"History: {my_observer.history}")


if __name__ == "__main__":
    main()

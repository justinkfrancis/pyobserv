from pyobserv import Observable, Observer
import time


def test_creation_and_trigger():
    class Sensor(Observable):
        def __init__(self):
            super().__init__()
            self.value = 0
            self.name = "Sensor"

        def run(self):
            for i in range(10):
                self.value += 1
                self.trigger_observers(self.name, self.value)
                time.sleep(0.01)

    class Subscriber2x(Observer):
        def __init__(self):
            super().__init__()
            self.new_value = 0
            self.history = []

        def callback(self, data):
            self.new_value = data * 2
            self.history.append(data)

    class Subscriber3x(Observer):
        def __init__(self):
            super().__init__()
            self.new_value = 0

        def callback(self, data):
            self.new_value = data * 3

    sensor = Sensor()
    subscriber = Subscriber2x()
    subscriber2 = Subscriber3x()

    subscriber.register_event_and_cb(sensor.name, subscriber.callback)
    subscriber2.register_event_and_cb(sensor.name, subscriber2.callback)
    sensor.add_observer(subscriber)
    sensor.add_observer(subscriber2)

    sensor.run()
    assert subscriber.new_value == 20
    assert subscriber2.new_value == 30
    assert subscriber.history == [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


def test_threaded_observable():
    class Sensor(Observable):
        def __init__(self):
            super().__init__()
            self.value = 0
            self.value2 = 0

        def run(self):
            for i in range(10):
                self.value += 1
                self.trigger_observers("value1", self.value)
                time.sleep(0.01)

        def run2(self):
            for i in range(10):
                self.value2 += 2
                self.trigger_observers("value2", self.value2)
                time.sleep(0.01)

    class Subscriber(Observer):
        def __init__(self):
            super().__init__()
            self.new_value = 0
            self.new_value2 = 0

        def callback(self, data):
            self.new_value = data * 2

        def callback2(self, data):
            self.new_value2 = data * 3

    sensor = Sensor()
    subscriber = Subscriber()
    sensor.add_observer(subscriber)

    subscriber.register_event_and_cb("value1", subscriber.callback)
    subscriber.register_event_and_cb("value2", subscriber.callback2)

    import threading

    t = threading.Thread(target=sensor.run)
    t2 = threading.Thread(target=sensor.run2)
    t.start()
    t2.start()

    t.join()
    t2.join()

    assert subscriber.new_value == 20
    assert subscriber.new_value2 == 60


def test_two_kwargs():
    class Sensor(Observable):
        def __init__(self):
            super().__init__()
            self.value = 0

        def run(self):
            self.value = 1
            self.trigger_observers(event="sensor", data=self.value)

    class Subscriber(Observer):
        def __init__(self):
            super().__init__()
            self.new_value = 0

        def callback(self, data):
            self.new_value = data * 2

    sensor = Sensor()
    subscriber = Subscriber()
    sensor.add_observer(subscriber)
    subscriber.register_event_and_cb("sensor", subscriber.callback)

    sensor.run()

    assert subscriber.new_value == 2


def test_no_inheritance():
    observer = Observer()
    observable = Observable()
    observable.add_observer(observer)
    observer.data = 1

    @observer.register_event_and_cb("event")
    def foo(data):
        observer.data = data * 2

    observable.trigger_observers("event", data=1)

    assert observer.data == 2

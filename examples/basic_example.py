"""Basic example of using Observable and Observer class"""

from pyobserv import Observable, Observer


def main():
    """Create Observer and Observable and trigger event"""
    observer = Observer()
    observable = Observable()

    @observer.register_event_and_cb("event")
    def callback(data):
        print(f"Observer saw {data}")

    observable.add_observer(observer)
    observable.trigger_observers("event", "foo")
    observable.trigger_observers("event", 1)
    observable.trigger_observers("event", None)


if __name__ == "__main__":
    main()

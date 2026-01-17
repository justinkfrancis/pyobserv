import pytest
from pyobserv import Observer, Observable
import queue


def test_add_observer_success():
    observable = Observable()
    observer = Observer()

    observable.add_observer(observer)

    assert observer in observable.observers


def test_add_observer_invalid_type():
    observable = Observable()

    class foo:
        pass

    observer = foo()
    with pytest.raises(TypeError):
        observable.add_observer(observer)


def test_add_observer_inheritance():
    observable = Observable()

    class bar(Observer):
        pass

    foo = bar()

    assert observable.add_observer(foo) is True
    assert foo in observable.observers


def test_add_observer_no_duplicates():
    observable = Observable()
    observer = Observer()

    observable.add_observer(observer)
    observable.add_observer(observer)

    assert observable.observers.count(observer) == 1


def test_add_observer_invalid_type():
    observable = Observable()

    with pytest.raises(TypeError):
        observable.add_observer("not an observer")


def test_remove_observer_existing():
    observable = Observable()
    observer = Observer()

    observable.add_observer(observer)
    observable.remove_observer(observer)

    assert observer not in observable.observers


def test_remove_observer_not_present_does_nothing():
    observable = Observable()
    observer = Observer()

    # Should not raise
    observable.remove_observer(observer)

    assert observable.observers == []


def test_trigger_observers_calls_on_trigger():
    observable = Observable()
    observer = Observer()
    observer.counter = 0

    observable.add_observer(observer)

    @observer.register_event_and_cb("event")
    def foo():
        observer.counter += 1

    observable.trigger_observers("event")
    assert observer.counter == 1
    observable.trigger_observers("event")
    assert observer.counter == 2
    observable.trigger_observers("event2")
    assert observer.counter == 2


def test_trigger_with_no_observers_does_not_fail():
    observable = Observable()

    # Should not raise any exception
    observable.trigger_observers("event")


def test_trigger_with_wrong_args():
    observable = Observable()
    observer = Observer()
    observable.add_observer(observer)

    @observer.register_event_and_cb("event")
    def foo():
        pass

    with pytest.raises(TypeError):
        observable.trigger_observers("event", 1, 2, 3)
    with pytest.raises(TypeError):
        observable.trigger_observers("event", 1, 2, 3, foo="bar")
    observable.trigger_observers("event")

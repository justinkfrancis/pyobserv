import pytest

from pyobserv import Observer


def test_method_call_noargs():
    observer = Observer()

    # Test no arguments
    data = []

    def foo():
        data.append("foo")

    observer.register_event_and_cb("test_noargs", foo)
    observer._on_trigger("test_noargs")

    assert data == ["foo"]


def test_method_call_arg():
    observer = Observer()

    # Test single *args argument
    data = []

    def bar(msg):
        data.append(msg)

    observer.register_event_and_cb("test_args", bar)
    observer._on_trigger("test_args", "bar")

    assert data == ["bar"]


def test_method_call_args():
    observer = Observer()

    # Test multiple *args arguments
    data = []

    def baz(*msg):
        for m in msg:
            data.append(m)

    observer.register_event_and_cb("test_m_args", baz)
    observer._on_trigger("test_m_args", "foo", "bar", "baz")

    assert data == ["foo", "bar", "baz"]

    # Pass no args into args function
    observer._on_trigger("test_m_args")
    assert data == ["foo", "bar", "baz"]


def test_method_call_multiple_func():
    observer = Observer()

    # Pass multiple functions to on_trigger
    data = []

    def foo():
        data.append("foo")

    def bar():
        data.append("bar")

    observer.register_event_and_cb("test_m_func", foo, bar)
    observer._on_trigger("test_m_func")
    assert data == ["foo", "bar"]


def test_get_all_events_method_call():
    observer = Observer()

    def foo():
        pass

    def bar():
        pass

    def baz():
        pass

    observer.register_event_and_cb("get_all_events", foo, bar)
    observer.register_event_and_cb("get_all_events2", baz)
    events = observer.get_all_events()
    assert events == ["get_all_events", "get_all_events2"]


def test_get_all_callbacks_method_call():
    observer = Observer()

    def foo():
        pass

    def bar():
        pass

    def baz():
        pass

    observer.register_event_and_cb("get_all_callbacks", foo, bar)
    observer.register_event_and_cb("get_all_callbacks", baz)

    callbacks = observer.get_all_callbacks("get_all_callbacks")
    assert callbacks == [foo, bar, baz]


def test_decorated():
    # Test without arguments
    observer = Observer()
    data = []

    @observer.register_event_and_cb("test_decorated_noargs")
    def foo():
        data.append("foo")

    observer._on_trigger("test_decorated_noargs")
    assert data == ["foo"]

    # Test returned function
    foo()
    assert data == ["foo", "foo"]


def test_decorated_arg():
    observer = Observer()

    # Test with arguments
    data = []

    @observer.register_event_and_cb("test_decorated_arg")
    def foo(bar):
        data.append(bar)

    observer._on_trigger("test_decorated_arg", "foo")

    assert data == ["foo"]


def test_decorated_many_events():
    observer = Observer()

    # Test two functions with same event
    data = []

    @observer.register_event_and_cb("test_decorated_many_events")
    def foo():
        data.append("foo")

    @observer.register_event_and_cb("test_decorated_many_events")
    def bar():
        data.append("bar")

    observer._on_trigger("test_decorated_many_events")
    assert data == ["foo", "bar"]


def test_decorated_args():
    observer = Observer()

    # Test with args
    data = []

    @observer.register_event_and_cb("test_decorated_args")
    def foo(*args):
        for arg in args:
            data.append(arg)

    observer._on_trigger("test_decorated_args", "foo", "bar", "baz")
    assert data == ["foo", "bar", "baz"]


def test_decorated_kwarg():
    observer = Observer()

    # Test with kwargs
    data = []

    @observer.register_event_and_cb("test_decorated_kwargs")
    def foo(*args, **kwargs):
        for key, value in kwargs.items():
            data.append(value)

    observer._on_trigger("test_decorated_kwargs", None, foo="bar")
    assert data == ["bar"]


def test_get_all_events_decorated():
    observer = Observer()

    @observer.register_event_and_cb("get_all_events")
    def foo():
        pass

    @observer.register_event_and_cb("get_all_events")
    def bar():
        pass

    @observer.register_event_and_cb("get_all_events2")
    def baz():
        pass

    events = observer.get_all_events()
    assert events == ["get_all_events", "get_all_events2"]


def test_get_all_callbacks_decorated():
    observer = Observer()

    @observer.register_event_and_cb("get_all_callbacks")
    def foo():
        pass

    @observer.register_event_and_cb("get_all_callbacks")
    def bar():
        pass

    callbacks = observer.get_all_callbacks("get_all_callbacks")
    assert callbacks == [foo, bar]


def test_get_events_and_callbacks_decorated():
    observer = Observer()

    @observer.register_event_and_cb("test")
    def foo():
        pass

    @observer.register_event_and_cb("test")
    def bar():
        pass

    @observer.register_event_and_cb("test2")
    def baz():
        pass

    events_and_callbacks = observer.get_all_events_and_callbacks()
    assert list(events_and_callbacks.keys()) == ["test", "test2"]
    assert events_and_callbacks["test"] == [foo, bar]
    assert events_and_callbacks["test2"] == [baz]


def test_remove_callbacks():
    observer = Observer()

    @observer.register_event_and_cb("test")
    def foo():
        pass

    @observer.register_event_and_cb("test")
    def bar():
        pass

    @observer.register_event_and_cb("test")
    def baz():
        pass

    @observer.register_event_and_cb("test")
    def baz2():
        pass

    @observer.register_event_and_cb("test2 ")
    def foobar():
        pass

    @observer.register_event_and_cb("test2")
    def foofoo():
        pass

    @observer.register_event_and_cb("test2")
    def foobaz():
        pass

    @observer.register_event_and_cb("test3")
    def foobar2():
        pass

    observer.unregister_event_or_cb("test", baz2)
    assert observer.get_all_callbacks("test") == [foo, bar, baz]

    observer.unregister_event_or_cb("test", foo, bar)
    assert observer.get_all_callbacks("test") == [baz]

    observer.unregister_event_or_cb("test2")
    assert observer.get_all_callbacks("test2") == []
    assert observer.get_all_callbacks("test3") == [foobar2]

    observer.unregister_event_or_cb()
    assert observer.get_all_events() == []


def test_remove_callbacks_raise_decorated():
    observer = Observer()

    with pytest.raises(LookupError):
        observer.unregister_event_or_cb("test")

    @observer.register_event_and_cb("test")
    def foo():
        pass

    def bar():
        pass

    with pytest.raises(LookupError):
        observer.unregister_event_or_cb("test", bar)


if __name__ == "__main__":
    pytest.main([__file__])

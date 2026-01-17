import typing as T
from collections import defaultdict

from pyobserv import Observer


class Observable:
    """
    Allows class to be extended to add and notify observers with an event
    trigger and data associated with that event in the form of args or
    kwargs.  Honus is on the user to ensure that args and kwargs passed by
    the trigger_observers function match the callback functions registered
    by the observer.
    """

    def __init__(self, *args, **kwargs) -> None:
        self.observers = []

    def add_observer(self, observer: Observer) -> bool:
        """
        Add an observer to the observers list, does not care about
        events.  Also prevents the same observer from being added twice

        :param observer: Observer object to be triggered by an event

        :return: Returns true if observer is added, false or TypeError
        otherwise
        """
        if not isinstance(observer, Observer):
            raise TypeError

        if observer not in self.observers:
            self.observers.append(observer)
            return True

        return False

    def remove_observer(self, observer: Observer):
        """
        Removes an observer from the observers list.

        :param observer: observer object to be removed

        :return: True if observer is removed, false otherwise
        """

        if observer in self.observers:
            self.observers.remove(observer)
            return True
        return False

    def trigger_observers(self, event: str, *args, **kwargs) -> None:
        """
        Triggers all the observers registered with an event to call their
        callbacks with optional args and kwargs arguments.  Honus is on the
        user to ensure that args and kwargs passed by the trigger_observers
        matches those callbacks registered by the observer.

        :param event: string representing the event to trigger the observers
        :param args: arguments to trigger from observable (typically data)
        :param kwargs: keyword arguments to trigger from observable (
        typically not used)

        :return: None
        """
        for observer in self.observers:
            observer._on_trigger(event, *args, **kwargs)

"""Observer Module"""

from collections import defaultdict
import typing as T


class Observer:
    """Class which notified by an Observable object. Handles all callbacks for
    individual events when triggered by an Observable object.  Honus is on
    the user to ensure that args from the Observable's trigger_observers for a
    given event match the callbacks registered functions args for that event.
    """

    def __init__(self) -> None:
        self._event_callbacks: T.Dict[str, T.List[T.Callable]] = defaultdict(
            list
        )

    def register_event_and_cb(
        self, event: str, *callbacks: T.Callable
    ) -> T.Callable:
        """
        Registers a function to be called when an event is triggered by the
        Observable object. Can be used directly or as a decorator.

        :param event: event to register from observable
        :param callbacks: decorator or list of callbacks to be called

        :return: original function passed my method or decorator
        """

        def wrapper(func):
            self._event_callbacks[event].append(func)
            # Ensures original function getting decorated is still callable
            return func

        if callbacks:
            # If callbacks are registered directly with a method call
            for cb in callbacks:
                wrapper(cb)
            return callbacks[0]

        # If callbacks are registered with a decorator
        return wrapper

    def _on_trigger(self, event: str, *args, **kwargs) -> bool:
        """
        Trigger method called by the Observable object.  Iterates through
        all callbacks registered for an event and calls them with arguments
        passed by observable.

        :param event: event to trigger from observable
        :param args: arguments to trigger from observable (typically data)
        :param kwargs: keyword arguments to trigger from observable (
        typically not used)

        :return: Boolean whether the trigger was successful or not
        """
        callbacks = list(self._event_callbacks.get(event, []))
        if not callbacks:
            return False

        for function in callbacks:
            function(*args, **kwargs)
        return True

    def unregister_event_or_cb(
        self, event: str, *callbacks: T.Callable
    ) -> None:
        """
        Will unregister all events and callbacks if event is None or false,
        unregister all callbacks for an event if no callbacks are passed,
        will unregister a single or multiple callbacks if callbacks are passed

        :param event: event to unregister from observable
        :param callbacks: single or multiple callbacks to unregisters

        :return: None
        """

        if not event:
            self._event_callbacks.clear()
            return

        if event not in self._event_callbacks:
            raise LookupError(f"Event: {event} not registered")

        if not callbacks:
            self._event_callbacks.pop(event)

        for func in callbacks:
            if func not in self._event_callbacks[event]:
                raise LookupError(
                    f"Function {func} not registered for " f"Event: {event}"
                )

            while func in self._event_callbacks[event]:
                self._event_callbacks[event].remove(func)
        return

    def get_all_events(self) -> T.List[str]:
        """
        Returns a list of all registered events
        """
        return list(self._event_callbacks.keys())

    def get_all_events_and_callbacks(self) -> T.Dict[str, T.List[T.Callable]]:
        """
        Returns a dict of all event strings as keys and callbacks associated
        with those events as values
        """
        events_and_callbacks = {}
        for event, callbacks in self._event_callbacks.items():
            events_and_callbacks[event] = list(callbacks)

        return events_and_callbacks

    def get_all_callbacks(self, event: str) -> T.List[T.Callable]:
        """
        Returns a list of all callbacks registered by the observer for a given
        event
        """
        return self._event_callbacks.get(event, [])

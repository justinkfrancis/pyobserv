"""PyObserv Module"""

# pyobserv/__init__.py
# pylint: disable=cyclic-import

from .observer import Observer
from .observable import Observable

__all__ = ["Observer", "Observable"]

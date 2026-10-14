"""Python bindings for the XRoboToolkit PC Service SDK.

Exposes XR device state streamed from a PICO headset via the XRoboToolkit
PC Service: controller poses and inputs, hand tracking, whole-body motion
tracking, and standalone motion trackers.
"""

from __future__ import annotations

import os
import sys


def _add_windows_dll_directory() -> None:
    """Make the bundled PXREARobotSDK.dll resolvable before importing _core."""
    if sys.platform != "win32":
        return
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)))


_add_windows_dll_directory()

from xrobotoolkit_sdk._core import *  # noqa: E402,F403

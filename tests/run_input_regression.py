"""Compile the real Python binding against a fake SDK, then exercise its API.

Run with Python >= 3.10 and a C++17 compiler on Linux/macOS:
    python tests/run_input_regression.py
Build products live in a temporary directory. The production SDK is not loaded.
"""

from __future__ import annotations

import ctypes
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import sysconfig
import tempfile
import threading
import unittest


ROOT = Path(__file__).resolve().parents[1]


def controller(marker=1, grip=1.0):
    return {
        "pose": f"{marker},0,0,0,0,0,1",
        "trigger": 0.25, "grip": grip, "menuButton": False,
        "axisX": 0.5, "axisY": -0.5, "axisClick": False,
        "primaryButton": True, "secondaryButton": False,
    }


def frame(marker=1, grip=1.0):
    return {
        "Controller": {"left": controller(marker, grip), "right": controller(marker, grip)},
        "Head": {"pose": f"{marker},0,0,0,0,0,1"}, "Hand": {},
        "timeStampNs": marker,
    }


class InputAPIRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="xrt-input-regression-")
        suffix = sysconfig.get_config_var("EXT_SUFFIX")
        extension = Path(cls.build_dir.name) / ("_core" + suffix)
        command = shlex.split(os.environ.get("CXX", "c++"))
        command += [
            "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-fPIC",
            "-DPy_LIMITED_API=0x030A0000",
            "-I" + sysconfig.get_path("include"),
            "-I" + str(ROOT / "vendor/include"),
            str(ROOT / "bindings/py_bindings.cpp"), str(ROOT / "tests/fake_sdk.cpp"),
            "-o", str(extension),
        ]
        if sys.platform == "darwin":
            # Vendor SDK supports Linux/Windows only. This macro affects the
            # fake ABI build, not the production library/platform selection.
            command += ["-DPXREACLIENTSDK_EXPORT=", "-dynamiclib", "-undefined", "dynamic_lookup"]
        else:
            command += ["-shared", "-pthread"]
        if os.environ.get("XRT_SANITIZE"):
            command += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
        print("Build:", shlex.join(command), flush=True)
        subprocess.run(command, check=True)
        spec = importlib.util.spec_from_file_location("_core", extension)
        cls.sdk = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.sdk)
        cls.native = ctypes.CDLL(str(extension))
        cls.native.xrt_test_emit_state.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        cls.native.xrt_test_emit_state.restype = ctypes.c_int
        cls.native.xrt_test_emit_event.argtypes = [ctypes.c_int, ctypes.c_char_p]
        cls.native.xrt_test_emit_event.restype = None
        cls.native.xrt_test_init_result.argtypes = [ctypes.c_int]
        cls.native.xrt_test_init_result.restype = None

    @classmethod
    def tearDownClass(cls):
        cls.sdk.close()
        cls.build_dir.cleanup()

    def setUp(self):
        self.sdk.close()
        self.native.xrt_test_init_result(0)
        self.sdk.init()
        self.event(1 << 2)

    def emit(self, device="headset-A", value=None, raw=None):
        if raw is None:
            raw = json.dumps({"value": json.dumps(frame() if value is None else value)})
        self.assertEqual(self.native.xrt_test_emit_state(device.encode(), raw.encode()), 0)

    def event(self, kind, device=None):
        self.native.xrt_test_emit_event(kind, None if device is None else device.encode())

    def assert_neutral(self):
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_headset_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_left_grip(), 0.0)
        self.assertEqual(self.sdk.get_right_grip(), 0.0)
        self.assertFalse(self.sdk.get_X_button())
        self.assertEqual(self.sdk.get_time_stamp_ns(), 0)

    def test_online_devices_keep_legacy_receiving_behavior(self):
        self.emit()
        self.emit("headset-B", frame(2, 0.5))
        self.assertEqual(self.sdk.get_left_controller_pose()[0], 2)
        self.assertEqual(self.sdk.get_right_grip(), 0.5)
        for name in ("set_device_id", "set_input_timeout_ms",
                     "is_input_valid", "get_input_snapshot"):
            self.assertFalse(hasattr(self.sdk, name))

    def test_unrelated_missing_does_not_clear_current_values(self):
        self.emit()
        self.event(1 << 5, "headset-B")
        self.assertEqual(self.sdk.get_left_grip(), 1.0)
        self.assertEqual(self.sdk.get_headset_pose()[0], 1)

    def test_missing_clears_only_domains_owned_by_that_device(self):
        self.emit()
        self.emit("headset-B", {
            "Controller": {}, "Hand": {},
            "Head": {"pose": "2,0,0,0,0,0,1"}, "timeStampNs": 2,
        })
        self.event(1 << 5, "headset-A")
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_grip(), 0.0)
        self.assertEqual(self.sdk.get_headset_pose()[0], 2)
        self.assertEqual(self.sdk.get_time_stamp_ns(), 2)
        self.event(1 << 5, "headset-B")
        self.assert_neutral()

    def test_target_missing_clears_all_modalities(self):
        value = frame()
        hand = {"scale": 1.0, "isActive": 1,
                "HandJointLocations": [{"p": "1,0,0,0,0,0,1"}] * 26}
        value["Hand"] = {"leftHand": hand, "rightHand": hand}
        value["Body"] = {"timeStampNs": 9, "joints": [
            {"p": "1,0,0,0,0,0,1", "va": "1,2,3,4,5,6",
             "wva": "6,5,4,3,2,1", "t": 9}]}
        value["Motion"] = {"timeStampNs": 8, "joints": [
            {"p": "1,0,0,0,0,0,1", "va": "1,2,3,4,5,6",
             "wva": "6,5,4,3,2,1", "sn": "tracker"}]}
        self.emit(value=value)
        self.assertTrue(self.sdk.is_body_data_available())
        self.assertEqual(self.sdk.num_motion_data_available(), 1)
        self.assertEqual(self.sdk.get_left_hand_is_active(), 1)
        self.event(1 << 5, "headset-A")
        self.assert_neutral()
        self.assertFalse(self.sdk.is_body_data_available())
        self.assertEqual(self.sdk.get_body_joints_pose(), [[0.0] * 7] * 24)
        self.assertEqual(self.sdk.get_body_joints_velocity(), [[0.0] * 6] * 24)
        self.assertEqual(self.sdk.get_body_joints_acceleration(), [[0.0] * 6] * 24)
        self.assertEqual(self.sdk.get_body_joints_timestamp(), [0] * 24)
        self.assertEqual(self.sdk.get_body_timestamp_ns(), 0)
        self.assertEqual(self.sdk.get_motion_tracker_pose(), [])
        self.assertEqual(self.sdk.get_motion_tracker_velocity(), [])
        self.assertEqual(self.sdk.get_motion_tracker_acceleration(), [])
        self.assertEqual(self.sdk.get_motion_tracker_serial_numbers(), [])
        self.assertEqual(self.sdk.get_motion_timestamp_ns(), 0)
        self.assertEqual(self.sdk.get_left_hand_is_active(), 0)
        self.assertEqual(self.sdk.get_right_hand_tracking_state(), [[0.0] * 7] * 26)

    def test_late_missing_frames_cannot_restore_values_before_online_event(self):
        self.emit()
        self.event(1 << 5, "headset-A")
        self.emit(value=frame(2))
        self.assert_neutral()
        self.event(1 << 4, "headset-A")
        self.assert_neutral()
        self.emit(value=frame(3))
        self.assertEqual(self.sdk.get_left_controller_pose()[0], 3)

    def test_service_disconnect_clears_and_blocks_late_frames_until_reconnect(self):
        self.emit()
        self.event(1 << 3)
        self.emit(value=frame(2))
        self.assert_neutral()
        self.event(1 << 2)
        self.assert_neutral()
        self.emit(value=frame(3))
        self.assertEqual(self.sdk.get_time_stamp_ns(), 3)

    def test_close_ignores_late_callbacks_and_is_idempotent(self):
        self.emit()
        self.sdk.close()
        count = self.native.xrt_test_close_calls()
        self.emit(value=frame(2))
        self.assert_neutral()
        self.sdk.close()
        self.assertEqual(self.native.xrt_test_close_calls(), count)
        self.sdk.init()
        self.assert_neutral()

    def test_invalid_side_clears_pose_and_input_without_affecting_valid_side(self):
        self.emit()
        value = frame(2)
        value["Controller"]["left"] = {"valid": False}
        value["Controller"]["right"]["valid"] = True
        self.emit(value=value)
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_left_grip(), 0.0)
        self.assertEqual(self.sdk.get_left_trigger(), 0.0)
        self.assertEqual(self.sdk.get_left_axis(), [0.0, 0.0])
        self.assertFalse(self.sdk.get_X_button())
        self.assertEqual(self.sdk.get_right_controller_pose()[0], 2)
        self.assertEqual(self.sdk.get_right_grip(), 1.0)
        self.emit(value=frame(3))
        self.assertEqual(self.sdk.get_left_controller_pose()[0], 3)

    def test_explicit_invalid_placeholder_is_not_cached(self):
        self.emit()
        value = frame(2)
        for side in ("left", "right"):
            value["Controller"][side]["valid"] = False
            value["Controller"][side]["pose"] = "0,0,0,0,0,0,1"
        value["Head"] = {"valid": False, "pose": "0,0,0,0,0,0,1"}
        self.emit(value=value)
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_headset_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_grip(), 0.0)

    def test_origin_identity_and_inactive_zero_hand_poses_are_not_disconnects(self):
        value = frame(0)
        value["Controller"]["left"]["valid"] = True
        value["Hand"] = {"leftHand": {"scale": 1.0, "isActive": 0,
            "HandJointLocations": [{"p": "0,0,0,0,0,0,0"}] * 26}}
        self.emit(value=value)
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 6 + [1.0])
        self.assertEqual(self.sdk.get_left_grip(), 1.0)

    def test_concurrent_late_callbacks_do_not_repopulate_missing_device(self):
        errors = []

        def producer():
            try:
                for marker in range(1, 201):
                    self.emit(value=frame(marker, marker / 200))
            except BaseException as error:
                errors.append(error)

        self.emit()
        thread = threading.Thread(target=producer)
        thread.start()
        self.event(1 << 5, "headset-A")
        thread.join(timeout=5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assert_neutral()

    def test_duplicate_init_does_not_clear_active_cache(self):
        self.emit()
        count = self.native.xrt_test_init_calls()
        with self.assertRaises(RuntimeError):
            self.sdk.init()
        self.assertEqual(self.native.xrt_test_init_calls(), count)
        self.assertEqual(self.sdk.get_left_grip(), 1.0)

    def test_init_failure_leaves_empty_cache(self):
        self.sdk.close()
        self.native.xrt_test_init_result(-1)
        with self.assertRaises(RuntimeError):
            self.sdk.init()
        self.emit()
        self.assert_neutral()


if __name__ == "__main__":
    unittest.main(verbosity=2)

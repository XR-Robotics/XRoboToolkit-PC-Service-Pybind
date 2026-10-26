# XRoboToolkit-PC-Service-Pybind

This project provides a python interface to extract XR state using XRoboToolkit-PC-Service sdk.

## Requirements

- Linux (x86_64 or aarch64, e.g. NVIDIA Orin) or Windows (x64), Python >= 3.10
- A running [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) with a connected PICO headset (at runtime)

The prebuilt `PXREARobotSDK` binaries are vendored in `vendor/`, so no manual
SDK download or build is needed.

## Installation

### From PyPI

```bash
uv add xrobotoolkit-sdk
# or
pip install xrobotoolkit-sdk
```

### From source

If you do not have [uv](https://docs.astral.sh/uv/) installed, run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then (requires a C++17 compiler; CMake >= 3.26 is downloaded automatically if
not installed):

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
uv sync
```

`uv sync` creates `.venv/` with any available Python >= 3.10 (pick one with
e.g. `uv sync --python 3.12`), builds the extension, and installs the package
into it. Run scripts with e.g. `uv run examples/example.py`.

To build distributable artifacts locally: `uv build`. The extension is built
against the CPython stable ABI, so the wheel (e.g.
`xrobotoolkit_sdk-1.1.0-cp310-abi3-win_amd64.whl`) installs on any CPython
>= 3.10 for the same OS and architecture, whichever Python version built it.
Wheels for Linux/Windows are also built in CI via cibuildwheel, see
`.github/workflows/wheels.yml`.

## Using the Python Bindings

The existing getters keep their signatures and return shapes. An SDK
`DeviceMissing` event clears values last written by that device; a service
disconnect or `close()` clears all cached values. Late frames from a reported
missing device are ignored until `DeviceFind`, and service frames are ignored
until `ServerConnect`. Reconnect events alone do not restore old values.

Controller or head payloads explicitly marked `valid: false` clear the
corresponding pose (and controller buttons/axes), rather than caching their
placeholder pose. Legacy payloads without `valid` are still accepted. Cleared
poses contain seven zeros, including a zero quaternion, and are **unavailable
data**, not a robot control target. A valid origin/identity pose is still accepted.

This uses SDK-reported events, not a new timeout or pose-based disconnect
heuristic. It does not select a headset or isolate simultaneous online devices.
`DeviceMissing` reflects the PC Service's offline decision and may be delayed
by its existing heartbeat policy.

**1. Get Controller and Headset Poses**

```python
import xrobotoolkit_sdk as xrt

xrt.init()

left_pose = xrt.get_left_controller_pose()
right_pose = xrt.get_right_controller_pose()
headset_pose = xrt.get_headset_pose()

print(f"Left Controller Pose: {left_pose}")
print(f"Right Controller Pose: {right_pose}")
print(f"Headset Pose: {headset_pose}")

xrt.close()
```

**2. Get Controller Inputs (Triggers, Grips, Buttons, Axes)**

```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Triggers and Grips
left_trigger = xrt.get_left_trigger()
right_grip = xrt.get_right_grip()
print(f"Left Trigger: {left_trigger}, Right Grip: {right_grip}")

# Buttons
a_button_pressed = xrt.get_A_button()
x_button_pressed = xrt.get_X_button()
print(f"A Button Pressed: {a_button_pressed}, X Button Pressed: {x_button_pressed}")

# Axes
left_axis = xrt.get_left_axis()
right_axis_click = xrt.get_right_axis_click()
print(f"Left Axis: {left_axis}, Right Axis Clicked: {right_axis_click}")

# Timestamp
timestamp = xrt.get_time_stamp_ns()
print(f"Current Timestamp (ns): {timestamp}")

xrt.close()
```

**3. Get hand tracking state**
```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Left Hand State
left_hand_tracking_state = xrt.get_left_hand_tracking_state()
print(f"Left Hand State: {left_hand_tracking_state}")

# Left Hand isActive
left_hand_is_active = xrt.get_left_hand_is_active()
print(f"Left Hand isActive: {left_hand_is_active}")

# Right Hand State
right_hand_tracking_state = xrt.get_right_hand_tracking_state()
print(f"Right Hand State: {right_hand_tracking_state}")

# Right Hand isActive
right_hand_is_active = xrt.get_right_hand_is_active()
print(f"Right Hand isActive: {right_hand_is_active}")

xrt.close()
```

**4. Get whole body motion tracking （please refer to this example when check Full Body tracking mode in UNITY app）**
```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Check if body tracking data is available
if xrt.is_body_data_available():
    # Get body joint poses (24 joints, 7 values each: x,y,z,qx,qy,qz,qw)
    body_poses = xrt.get_body_joints_pose()
    print(f"Body joints pose data: {body_poses}")
    
    # Get body joint velocities (24 joints, 6 values each: vx,vy,vz,wx,wy,wz)
    body_velocities = xrt.get_body_joints_velocity()
    print(f"Body joints velocity data: {body_velocities}")
    
    # Get body joint accelerations (24 joints, 6 values each: ax,ay,az,wax,way,waz)
    body_accelerations = xrt.get_body_joints_acceleration()
    print(f"Body joints acceleration data: {body_accelerations}")
    
    # Get IMU timestamps for each joint
    imu_timestamps = xrt.get_body_joints_timestamp()
    print(f"IMU timestamps: {imu_timestamps}")
    
    # Get body data timestamp
    body_timestamp = xrt.get_body_timestamp_ns()
    print(f"Body data timestamp: {body_timestamp}")
    
    # Example: Get specific joint data (Head joint is index 15)
    head_pose = body_poses[15]  # Head joint
    x, y, z, qx, qy, qz, qw = head_pose
    print(f"Head pose: Position({x:.3f}, {y:.3f}, {z:.3f}) Rotation({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
else:
    print("Body tracking data not available. Make sure:")
    print("1. PICO headset is connected")
    print("2. Body tracking is enabled in the control panel")
    print("3. At least two Pico Swift devices are connected and calibrated")

xrt.close()
```

**Body Joint Indices (similar to SMPL, 24 joints total):**
- 0: Pelvis, 1: Left Hip, 2: Right Hip, 3: Spine1, 4: Left Knee, 5: Right Knee
- 6: Spine2, 7: Left Ankle, 8: Right Ankle, 9: Spine3, 10: Left Foot, 11: Right Foot
- 12: Neck, 13: Left Collar, 14: Right Collar, 15: Head, 16: Left Shoulder, 17: Right Shoulder
- 18: Left Elbow, 19: Right Elbow, 20: Left Wrist, 21: Right Wrist, 22: Left Hand, 23: Right Hand

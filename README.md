# XRoboToolkit-PC-Service-Pybind

This project provides a python interface to extract XR state using XRoboToolkit-PC-Service sdk.

## Requirements

- [`pybind11`](https://github.com/pybind/pybind11)
- [`XRoboRoolkit PC Service`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service#)

## Building the Project
### Ubuntu 22.04
```
bash setup_ubuntu.sh
```

### Windows
- Compile the `PXREARobotSDK` under [`XRoboRoolkit PC Service`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service#)
- copy `PXREARobotSDK.lib` and `PXREARobotSDK.dll` to `lib` folder.
- install the package by
```
python setup.py install
```

## Using the Python Bindings

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
# Vendored PXREARobotSDK binaries

Prebuilt binaries and headers copied verbatim from
[XR-Robotics/XRoboToolkit-PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
at commit `85bac4dbc1fd5cef42c74a160d9c30aa3491f122`.

| Path | Upstream source |
| --- | --- |
| `include/PXREARobotSDK.h` | `RoboticsService/SDK/include/PXREARobotSDK.h` |
| `include/nlohmann/` | `RoboticsService/PXREARobotSDK/nlohmann/` |
| `linux_x86_64/libPXREARobotSDK.so` | `RoboticsService/SDK/linux/64/` (needs glibc >= 2.29) |
| `linux_aarch64/libPXREARobotSDK.so` | `RoboticsService/SDK/linux_aarch64/64/` (needs glibc >= 2.34) |
| `windows_amd64/PXREARobotSDK.{dll,lib}` | `RoboticsService/SDK/win/64/` |

To update: clone upstream, copy the files listed above, and record the new
commit hash here. If the glibc requirements of the Linux libraries change,
adjust the `manylinux-*-image` settings in `pyproject.toml` accordingly.

# 3.26: FindPython's Development.SABIModule / python_add_library(USE_SABI).
cmake_minimum_required(VERSION 3.26)

project(xrobotoolkit_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# When building an abi3 wheel (wheel.py-api in pyproject.toml), scikit-build-core
# sets SKBUILD_SABI_COMPONENT=Development.SABIModule and SKBUILD_SABI_VERSION=3.10.
# Both are empty on interpreters without a stable ABI (e.g. free-threaded
# CPython), which then get a regular version-specific extension.
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module ${SKBUILD_SABI_COMPONENT})

# --- Select the vendored prebuilt PXREARobotSDK for the target platform ---
# Binaries are checked into vendor/ (see vendor/README.md for provenance).
if(WIN32)
  set(XRT_VENDOR_DIR "${PROJECT_SOURCE_DIR}/vendor/windows_amd64")
  set(XRT_VENDOR_IMPLIB "${XRT_VENDOR_DIR}/PXREARobotSDK.lib")
  set(XRT_VENDOR_RUNTIME "${XRT_VENDOR_DIR}/PXREARobotSDK.dll")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(XRT_VENDOR_DIR "${PROJECT_SOURCE_DIR}/vendor/linux_aarch64")
  else()
    set(XRT_VENDOR_DIR "${PROJECT_SOURCE_DIR}/vendor/linux_x86_64")
  endif()
  set(XRT_VENDOR_IMPLIB "${XRT_VENDOR_DIR}/libPXREARobotSDK.so")
  set(XRT_VENDOR_RUNTIME "${XRT_VENDOR_DIR}/libPXREARobotSDK.so")
else()
  message(FATAL_ERROR
    "Unsupported platform '${CMAKE_SYSTEM_NAME}'. PXREARobotSDK ships prebuilt "
    "binaries for Linux (x86_64, aarch64) and Windows (amd64) only.")
endif()

if(SKBUILD_SABI_VERSION)
  python_add_library(_core MODULE WITH_SOABI USE_SABI ${SKBUILD_SABI_VERSION} bindings/py_bindings.cpp)
else()
  python_add_library(_core MODULE WITH_SOABI bindings/py_bindings.cpp)
endif()

# Export only PyInit__core (as pybind11_add_module did), so the bundled
# nlohmann/json symbols cannot interpose with those of the vendored SDK.
set_target_properties(_core PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)

target_include_directories(_core PRIVATE "${PROJECT_SOURCE_DIR}/vendor/include")
target_link_libraries(_core PRIVATE "${XRT_VENDOR_IMPLIB}")

# The extension and the vendored shared library are installed side by side in
# the package directory; $ORIGIN makes the dynamic linker look there first.
# (On Windows, __init__.py calls os.add_dll_directory instead.)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set_target_properties(_core PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

install(TARGETS _core LIBRARY DESTINATION xrobotoolkit_sdk)
install(FILES "${XRT_VENDOR_RUNTIME}" DESTINATION xrobotoolkit_sdk)

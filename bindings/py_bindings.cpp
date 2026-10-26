// Python.h must come first. Py_LIMITED_API is defined by CMake for abi3 builds.
#include <Python.h>

#include <thread>
#include <iostream>
#include <mutex>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include "PXREARobotSDK.h"


using json = nlohmann::json;

std::array<double, 7> LeftControllerPose;
std::array<double, 7> RightControllerPose;
std::array<double, 7> HeadsetPose;

std::array<std::array<double, 7>, 26> LeftHandTrackingState;
double LeftHandScale = 1.0;
int LeftHandIsActive = 0;
std::array<std::array<double, 7>, 26> RightHandTrackingState;
double RightHandScale = 1.0;
int RightHandIsActive = 0;

// Whole body motion data - 24 joints for body tracking
std::array<std::array<double, 7>, 24> BodyJointsPose;  // Position and rotation for each joint
std::array<std::array<double, 6>, 24> BodyJointsVelocity;  // Velocity and angular velocity for each joint
std::array<std::array<double, 6>, 24> BodyJointsAcceleration;  // Acceleration and angular acceleration for each joint
std::array<int64_t, 24> BodyJointsTimestamp;  // IMU timestamp for each joint
int64_t BodyTimeStampNs = 0;  // Body data timestamp
bool BodyDataAvailable = false;  // Flag to indicate if body data is available

std::array<std::array<double, 7>, 3> MotionTrackerPose;  // Position and rotation for each joint
std::array<std::array<double, 6>, 3> MotionTrackerVelocity;  // Velocity and angular velocity for each joint
std::array<std::array<double, 6>, 3> MotionTrackerAcceleration;  // Acceleration and angular acceleration for each joint
std::array<std::string, 3> MotionTrackerSerialNumbers;  // Serial numbers of the motion trackers
int64_t MotionTimeStampNs = 0;  // Motion data timestamp
int NumMotionDataAvailable = 0;  // number of motion trackers


bool LeftMenuButton;
double LeftTrigger;
double LeftGrip;
std::array<double, 2> LeftAxis{0.0, 0.0};
bool LeftAxisClick;
bool LeftPrimaryButton;
bool LeftSecondaryButton;

bool RightMenuButton;
double RightTrigger;
double RightGrip;
std::array<double, 2> RightAxis{0.0, 0.0};
bool RightAxisClick;
bool RightPrimaryButton;
bool RightSecondaryButton;

int64_t TimeStampNs;

std::mutex leftMutex;
std::mutex rightMutex;
std::mutex headsetPoseMutex;
std::mutex timestampMutex;
std::mutex leftHandMutex;
std::mutex rightHandMutex;
std::mutex bodyMutex;  // Mutex for body tracking data
std::mutex motionMutex;

// Serialize callbacks with cache invalidation, without holding this mutex
// while SDK init/deinit may start or join callback threads.
std::mutex callbackMutex;
std::mutex sdkLifecycleMutex;
bool initialized = false;
bool serverDisconnected = false;
std::unordered_set<std::string> missingDevices;

enum CacheDomain { Left, Right, Head, Timestamp, LeftHand, RightHand, Body, Motion };
// Ownership is only for clearing a disconnected device's values. This does
// not select a device or change how frames from online devices are received.
std::array<std::string, 8> cacheOwners;

// Caller holds callbackMutex. Existing getter locks and return shapes stay.
void clearCachedInput(const std::string* device = nullptr) {
    std::scoped_lock lock(leftMutex, rightMutex, headsetPoseMutex, timestampMutex,
                          leftHandMutex, rightHandMutex, bodyMutex, motionMutex);
    auto clearDomain = [&](CacheDomain domain) {
        if (device && cacheOwners[domain] != *device) {
            return false;
        }
        cacheOwners[domain].clear();
        return true;
    };
    if (clearDomain(Left)) {
        LeftControllerPose = {};
        LeftTrigger = LeftGrip = 0.0;
        LeftAxis = {};
        LeftMenuButton = LeftAxisClick = LeftPrimaryButton = LeftSecondaryButton = false;
    }
    if (clearDomain(Right)) {
        RightControllerPose = {};
        RightTrigger = RightGrip = 0.0;
        RightAxis = {};
        RightMenuButton = RightAxisClick = RightPrimaryButton = RightSecondaryButton = false;
    }
    if (clearDomain(Head)) HeadsetPose = {};
    if (clearDomain(Timestamp)) TimeStampNs = 0;
    if (clearDomain(LeftHand)) {
        LeftHandTrackingState = {};
        LeftHandScale = 1.0;
        LeftHandIsActive = 0;
    }
    if (clearDomain(RightHand)) {
        RightHandTrackingState = {};
        RightHandScale = 1.0;
        RightHandIsActive = 0;
    }
    if (clearDomain(Body)) {
        BodyJointsPose = {};
        BodyJointsVelocity = {};
        BodyJointsAcceleration = {};
        BodyJointsTimestamp = {};
        BodyTimeStampNs = 0;
        BodyDataAvailable = false;
    }
    if (clearDomain(Motion)) {
        MotionTrackerPose = {};
        MotionTrackerVelocity = {};
        MotionTrackerAcceleration = {};
        MotionTrackerSerialNumbers = {};
        MotionTimeStampNs = 0;
        NumMotionDataAvailable = 0;
    }
}


std::array<double, 7> stringToPoseArray(const std::string& poseStr) {
    std::array<double, 7> result{0};
    std::stringstream ss(poseStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 7) {
        result[i++] = std::stod(value);
    }
    return result;
}

std::array<double, 6> stringToVelocityArray(const std::string& velocityStr) {
    std::array<double, 6> result{0};
    std::stringstream ss(velocityStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 6) {
        result[i++] = std::stod(value);
    }
    return result;
}

void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
    std::lock_guard<std::mutex> callbackLock(callbackMutex);
    if (!initialized) {
        return;
    }
    switch (type)
    {
    case PXREAServerConnect:
        serverDisconnected = false;
        std::cout << "server connect\n" << std::endl;
        break;
    case PXREAServerDisconnect:
        serverDisconnected = true;
        clearCachedInput();
        std::cout << "server disconnect\n" << std::endl;
        break;
    case PXREADeviceFind:
        if (userData) {
            missingDevices.erase(static_cast<const char*>(userData));
        }
        std::cout << "device found\n" << std::endl;
        break;
    case PXREADeviceMissing:
        if (userData) {
            const std::string device(static_cast<const char*>(userData));
            missingDevices.insert(device);
            clearCachedInput(&device);
        } else {
            // Without an identity, conservatively invalidate all cached input.
            clearCachedInput();
        }
        std::cout << "device missing\n" << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect\n" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson: {
        if (serverDisconnected || !userData) {
            break;
        }
        auto& dsj = *((PXREADevStateJson*)userData);
        const std::string device(dsj.devID,
            std::find(std::begin(dsj.devID), std::end(dsj.devID), '\0'));
        if (missingDevices.count(device)) {
            break;  // A late frame cannot revive an SDK-reported offline device.
        }

        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(leftMutex);
                        cacheOwners[Left] = device;
                        if (left.contains("valid") && !left.at("valid").get<bool>()) {
                            LeftControllerPose = {};
                            LeftTrigger = LeftGrip = 0.0;
                            LeftAxis = {};
                            LeftMenuButton = LeftAxisClick = LeftPrimaryButton = LeftSecondaryButton = false;
                        } else {
                            LeftControllerPose = stringToPoseArray(left["pose"].get<std::string>());
                            LeftTrigger = left["trigger"].get<double>();
                            LeftGrip = left["grip"].get<double>();
                            LeftMenuButton = left["menuButton"].get<bool>();
                            LeftAxis[0] = left["axisX"].get<double>();
                            LeftAxis[1] = left["axisY"].get<double>();
                            LeftAxisClick = left["axisClick"].get<bool>();
                            LeftPrimaryButton = left["primaryButton"].get<bool>();
                            LeftSecondaryButton = left["secondaryButton"].get<bool>();
                        }
                    }
                }
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightMutex);
                        cacheOwners[Right] = device;
                        if (right.contains("valid") && !right.at("valid").get<bool>()) {
                            RightControllerPose = {};
                            RightTrigger = RightGrip = 0.0;
                            RightAxis = {};
                            RightMenuButton = RightAxisClick = RightPrimaryButton = RightSecondaryButton = false;
                        } else {
                            RightControllerPose = stringToPoseArray(right["pose"].get<std::string>());
                            RightTrigger = right["trigger"].get<double>();
                            RightGrip = right["grip"].get<double>();
                            RightMenuButton = right["menuButton"].get<bool>();
                            RightAxis[0] = right["axisX"].get<double>();
                            RightAxis[1] = right["axisY"].get<double>();
                            RightAxisClick = right["axisClick"].get<bool>();
                            RightPrimaryButton = right["primaryButton"].get<bool>();
                            RightSecondaryButton = right["secondaryButton"].get<bool>();
                        }
                    }
                }
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        cacheOwners[Head] = device;
                        HeadsetPose = headset.contains("valid") && !headset.at("valid").get<bool>()
                            ? std::array<double, 7>{}
                            : stringToPoseArray(headset["pose"].get<std::string>());
                    }
                }
                if (value.contains("timeStampNs")) {
                    std::lock_guard<std::mutex> lock(timestampMutex);
                    cacheOwners[Timestamp] = device;
                    TimeStampNs = value["timeStampNs"].get<int64_t>();
                }
                if (value["Hand"].contains("leftHand")) {
                    auto& leftHand = value["Hand"]["leftHand"];
                    {
                        std::lock_guard<std::mutex> lock(leftHandMutex);
                        cacheOwners[LeftHand] = device;

                        LeftHandScale = leftHand["scale"].get<double>();
                        LeftHandIsActive = leftHand["isActive"].get<int>();
                        for (int i = 0; i < 26; i++) {
                            LeftHandTrackingState[i] = stringToPoseArray(leftHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
                if (value["Hand"].contains("rightHand")) {
                    auto& rightHand = value["Hand"]["rightHand"];
                    {
                        std::lock_guard<std::mutex> lock(rightHandMutex);
                        cacheOwners[RightHand] = device;
                        RightHandScale = rightHand["scale"].get<double>();
                        RightHandIsActive = rightHand["isActive"].get<int>();
                        for (int i = 0; i < 26; i++) {
                            RightHandTrackingState[i] = stringToPoseArray(rightHand["HandJointLocations"][i]["p"].get<std::string>());
                        }
                    }
                }
                // Parse Body data for whole body motion capture
                if (value.contains("Body")) {
                    auto& body = value["Body"];
                    {
                        std::lock_guard<std::mutex> lock(bodyMutex);
                        cacheOwners[Body] = device;

                        if (body.contains("timeStampNs")) {
                            BodyTimeStampNs = body["timeStampNs"].get<int64_t>();
                        }

                        if (body.contains("joints") && body["joints"].is_array()) {
                            auto joints = body["joints"];
                            int jointCount = std::min(static_cast<int>(joints.size()), 24);

                            for (int i = 0; i < jointCount; i++) {
                                auto& joint = joints[i];

                                // Parse pose (position and rotation)
                                if (joint.contains("p")) {
                                    BodyJointsPose[i] = stringToPoseArray(joint["p"].get<std::string>());
                                }

                                // Parse velocity and angular velocity
                                if (joint.contains("va")) {
                                    BodyJointsVelocity[i] = stringToVelocityArray(joint["va"].get<std::string>());
                                }

                                // Parse acceleration and angular acceleration
                                if (joint.contains("wva")) {
                                    BodyJointsAcceleration[i] = stringToVelocityArray(joint["wva"].get<std::string>());
                                }

                                // Parse IMU timestamp
                                if (joint.contains("t")) {
                                    BodyJointsTimestamp[i] = joint["t"].get<int64_t>();
                                }
                            }

                            BodyDataAvailable = true;
                        }
                    }
                }
                //parse individual tracker data
                if (value.contains("Motion")) {
                    auto& motion = value["Motion"];
                    {
                        std::lock_guard<std::mutex> lock(motionMutex);
                        cacheOwners[Motion] = device;
                        if (motion.contains("timeStampNs")) {
                            MotionTimeStampNs = motion["timeStampNs"].get<int64_t>();
                        }
                        if (motion.contains("joints") && motion["joints"].is_array()) {
                            auto joints = motion["joints"];
                            NumMotionDataAvailable = std::min(static_cast<int>(joints.size()), 3);

                            for (int i = 0; i < NumMotionDataAvailable; i++) {
                                auto& joint = joints[i];

                                // Parse pose (position and rotation)
                                if (joint.contains("p")) {
                                    MotionTrackerPose[i] = stringToPoseArray(joint["p"].get<std::string>());
                                }

                                // Parse velocity and angular velocity
                                if (joint.contains("va")) {
                                    MotionTrackerVelocity[i] = stringToVelocityArray(joint["va"].get<std::string>());
                                }

                                // Parse acceleration and angular acceleration
                                if (joint.contains("wva")) {
                                    MotionTrackerAcceleration[i] = stringToVelocityArray(joint["wva"].get<std::string>());
                                }

                                if (joint.contains("sn")) {
                                    MotionTrackerSerialNumbers[i] = joint["sn"].get<std::string>();
                                }
                            }

                        }
                    }
                }
            }
        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
            break;
    }
    }
}

void init() {
    std::lock_guard<std::mutex> sdkLock(sdkLifecycleMutex);
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        if (initialized) {
            throw std::runtime_error("PXREARobot SDK is already initialized");
        }
        clearCachedInput();
        missingDevices.clear();
        serverDisconnected = false;
        initialized = true;
    }
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        std::lock_guard<std::mutex> lock(callbackMutex);
        initialized = false;
        clearCachedInput();
        throw std::runtime_error("PXREAInit failed");
    }
}

void deinit() {
    std::lock_guard<std::mutex> sdkLock(sdkLifecycleMutex);
    bool wasInitialized;
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        wasInitialized = initialized;
        initialized = false;
        clearCachedInput();
        missingDevices.clear();
    }
    if (wasInitialized) {
        PXREADeinit();
    }
}

std::array<double, 7> getLeftControllerPose() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftControllerPose;
}

std::array<double, 7> getRightControllerPose() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightControllerPose;
}

std::array<double, 7> getHeadsetPose() {
    std::lock_guard<std::mutex> lock(headsetPoseMutex);
    return HeadsetPose;
}

double getLeftTrigger() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftTrigger;
}

double getLeftGrip() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftGrip;
}

double getRightTrigger() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightTrigger;
}

double getRightGrip() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightGrip;
}

bool getLeftMenuButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftMenuButton;
}

bool getRightMenuButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightMenuButton;
}

bool getLeftAxisClick() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxisClick;
}

bool getRightAxisClick() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxisClick;
}

std::array<double, 2> getLeftAxis() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftAxis;
}


std::array<double, 2> getRightAxis() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightAxis;
}

bool getLeftPrimaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftPrimaryButton;
}

bool getRightPrimaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightPrimaryButton;
}

bool getLeftSecondaryButton() {
    std::lock_guard<std::mutex> lock(leftMutex);
    return LeftSecondaryButton;
}

bool getRightSecondaryButton() {
    std::lock_guard<std::mutex> lock(rightMutex);
    return RightSecondaryButton;
}

int64_t getTimeStampNs() {
    std::lock_guard<std::mutex> lock(timestampMutex);
    return TimeStampNs;
}

std::array<std::array<double, 7>, 26> getLeftHandTrackingState() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandTrackingState;
}

int getLeftHandScale() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandScale;
}

int getLeftHandIsActive() {
    std::lock_guard<std::mutex> lock(leftHandMutex);
    return LeftHandIsActive;
}

std::array<std::array<double, 7>, 26> getRightHandTrackingState() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandTrackingState;
}

int getRightHandScale() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandScale;
}

int getRightHandIsActive() {
    std::lock_guard<std::mutex> lock(rightHandMutex);
    return RightHandIsActive;
}

// Body tracking functions
bool isBodyDataAvailable() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyDataAvailable;
}

std::array<std::array<double, 7>, 24> getBodyJointsPose() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsPose;
}

std::array<std::array<double, 6>, 24> getBodyJointsVelocity() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsVelocity;
}

std::array<std::array<double, 6>, 24> getBodyJointsAcceleration() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsAcceleration;
}

std::array<int64_t, 24> getBodyJointsTimestamp() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyJointsTimestamp;
}

int64_t getBodyTimeStampNs() {
    std::lock_guard<std::mutex> lock(bodyMutex);
    return BodyTimeStampNs;
}

int numMotionDataAvailable() {
    std::lock_guard<std::mutex> lock(motionMutex);
    return NumMotionDataAvailable;
}

std::vector<std::array<double, 7>> getMotionTrackerPose() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 7>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerPose[i]);
    }
    return result;
}

std::vector<std::array<double, 6>> getMotionTrackerVelocity() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 6>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerVelocity[i]);
    }
    return result;
}

std::vector<std::array<double, 6>> getMotionTrackerAcceleration() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::array<double, 6>> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerAcceleration[i]);
    }
    return result;
}

std::vector<std::string> getMotionTrackerSerialNumbers() {
    std::lock_guard<std::mutex> lock(motionMutex);
    std::vector<std::string> result;
    for (int i = 0; i < NumMotionDataAvailable; i++) {
        result.push_back(MotionTrackerSerialNumbers[i]);
    }
    return result;
}

int64_t getMotionTimeStampNs() {
    std::lock_guard<std::mutex> lock(motionMutex);
    return MotionTimeStampNs;
}

int DeviceControlJsonWrapper(const std::string& dev_id, const std::string& json_str) {
    const int rc = PXREADeviceControlJson(dev_id.c_str(), json_str.c_str());
    if (rc != 0) {
        throw std::runtime_error("device_control_json failed");
    }
    return rc; // 0
}

int SendBytesToDeviceWrapper(const std::string& dev_id, const std::string& blob) {
    const int rc = PXREASendBytesToDevice(dev_id.c_str(), blob.data(), static_cast<unsigned>(blob.size()));
    if (rc != 0) {
        throw std::runtime_error("send_bytes_to_device failed");
    }
    return rc; // 0
}


// ---------------------------------------------------------------------------
// Python module
//
// Written against the CPython Limited API instead of pybind11, which cannot
// target it: the resulting abi3 wheel installs on every CPython >= 3.10. The
// conversions match what pybind11 produced (std::array/std::vector -> list).
// ---------------------------------------------------------------------------

namespace {

template <typename T>
PyObject* toPython(const T& value) {
    if constexpr (std::is_same_v<T, bool>) {
        return PyBool_FromLong(value);
    } else if constexpr (std::is_integral_v<T>) {
        return PyLong_FromLongLong(value);
    } else if constexpr (std::is_floating_point_v<T>) {
        return PyFloat_FromDouble(value);
    } else if constexpr (std::is_same_v<T, std::string>) {
        return PyUnicode_FromStringAndSize(value.data(), static_cast<Py_ssize_t>(value.size()));
    } else {
        PyObject* list = PyList_New(static_cast<Py_ssize_t>(value.size()));
        if (list == nullptr) {
            return nullptr;
        }
        Py_ssize_t i = 0;
        for (const auto& item : value) {
            PyObject* element = toPython(item);
            if (element == nullptr) {
                Py_DECREF(list);
                return nullptr;
            }
            PyList_SetItem(list, i++, element);  // steals the reference
        }
        return list;
    }
}

// C++ exceptions must not unwind into the interpreter; raise them as Python
// exceptions the way pybind11 did.
template <typename Body>
PyObject* translateExceptions(Body body) {
    try {
        return body();
    } catch (const std::bad_alloc&) {
        return PyErr_NoMemory();
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

template <auto Fn>
PyObject* callNoArgs(PyObject* /*module*/, PyObject* /*unused*/) {
    return translateExceptions([]() -> PyObject* {
        if constexpr (std::is_void_v<decltype(Fn())>) {
            Fn();
            Py_RETURN_NONE;
        } else {
            return toPython(Fn());
        }
    });
}

// str (UTF-8 encoded), bytes or bytearray -> std::string, like pybind11's
// std::string caster. Deliberately not PyArg_ParseTuple's "s#"/"y#": when
// compiled against Python >= 3.13 headers, '#' formats raise SystemError
// (PY_SSIZE_T_CLEAN) once the abi3 module runs on Python 3.10-3.12.
bool toStdString(PyObject* obj, std::string& out) {
    const char* data;
    Py_ssize_t size;
    if (PyUnicode_Check(obj)) {
        data = PyUnicode_AsUTF8AndSize(obj, &size);
        if (data == nullptr) {
            return false;
        }
    } else if (PyBytes_Check(obj)) {
        char* buffer;
        if (PyBytes_AsStringAndSize(obj, &buffer, &size) < 0) {
            return false;
        }
        data = buffer;
    } else if (PyByteArray_Check(obj)) {
        data = PyByteArray_AsString(obj);
        size = PyByteArray_Size(obj);
    } else {
        PyErr_Format(PyExc_TypeError, "expected str, bytes or bytearray, got %R", reinterpret_cast<PyObject*>(Py_TYPE(obj)));
        return false;
    }
    out.assign(data, static_cast<size_t>(size));
    return true;
}

PyObject* pyDeviceControlJson(PyObject* /*module*/, PyObject* args) {
    PyObject* devId;
    PyObject* jsonStr;
    if (!PyArg_ParseTuple(args, "OO:device_control_json", &devId, &jsonStr)) {
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        std::string devIdValue;
        std::string jsonValue;
        if (!toStdString(devId, devIdValue) || !toStdString(jsonStr, jsonValue)) {
            return nullptr;
        }
        return toPython(DeviceControlJsonWrapper(devIdValue, jsonValue));
    });
}

PyObject* pySendBytesToDevice(PyObject* /*module*/, PyObject* args) {
    PyObject* devId;
    PyObject* data;  // "S": bytes only, like pybind11's py::bytes
    if (!PyArg_ParseTuple(args, "OS:send_bytes_to_device", &devId, &data)) {
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        std::string devIdValue;
        std::string blob;
        if (!toStdString(devId, devIdValue) || !toStdString(data, blob)) {
            return nullptr;
        }
        return toPython(SendBytesToDeviceWrapper(devIdValue, blob));
    });
}

// Docstrings keep pybind11's layout: "name() -> type", blank line, description.
#define XRT_FUNCTION(name, fn, returns, doc) \
    { name, callNoArgs<fn>, METH_NOARGS, name "() -> " returns "\n\n" doc }

PyMethodDef kMethods[] = {
    XRT_FUNCTION("init", init, "None", "Initialize the PXREARobot SDK."),
    XRT_FUNCTION("close", deinit, "None", "Deinitialize the PXREARobot SDK."),
    XRT_FUNCTION("get_left_controller_pose", getLeftControllerPose, "list[float]", "Get the left controller pose."),
    XRT_FUNCTION("get_right_controller_pose", getRightControllerPose, "list[float]", "Get the right controller pose."),
    XRT_FUNCTION("get_headset_pose", getHeadsetPose, "list[float]", "Get the headset pose."),
    XRT_FUNCTION("get_left_trigger", getLeftTrigger, "float", "Get the left trigger value."),
    XRT_FUNCTION("get_left_grip", getLeftGrip, "float", "Get the left grip value."),
    XRT_FUNCTION("get_right_trigger", getRightTrigger, "float", "Get the right trigger value."),
    XRT_FUNCTION("get_right_grip", getRightGrip, "float", "Get the right grip value."),
    XRT_FUNCTION("get_left_menu_button", getLeftMenuButton, "bool", "Get the left menu button state."),
    XRT_FUNCTION("get_right_menu_button", getRightMenuButton, "bool", "Get the right menu button state."),
    XRT_FUNCTION("get_left_axis_click", getLeftAxisClick, "bool", "Get the left axis click state."),
    XRT_FUNCTION("get_right_axis_click", getRightAxisClick, "bool", "Get the right axis click state."),
    XRT_FUNCTION("get_left_axis", getLeftAxis, "list[float]", "Get the left axis values (x, y)."),
    XRT_FUNCTION("get_right_axis", getRightAxis, "list[float]", "Get the right axis values (x, y)."),
    XRT_FUNCTION("get_X_button", getLeftPrimaryButton, "bool", "Get the left primary button state."),
    XRT_FUNCTION("get_A_button", getRightPrimaryButton, "bool", "Get the right primary button state."),
    XRT_FUNCTION("get_Y_button", getLeftSecondaryButton, "bool", "Get the left secondary button state."),
    XRT_FUNCTION("get_B_button", getRightSecondaryButton, "bool", "Get the right secondary button state."),
    XRT_FUNCTION("get_time_stamp_ns", getTimeStampNs, "int", "Get the timestamp in nanoseconds."),
    XRT_FUNCTION("get_left_hand_tracking_state", getLeftHandTrackingState, "list[list[float]]", "Get the left hand state."),
    XRT_FUNCTION("get_right_hand_tracking_state", getRightHandTrackingState, "list[list[float]]", "Get the right hand state."),
    XRT_FUNCTION("get_left_hand_is_active", getLeftHandIsActive, "int", "Get the left hand tracking quality (0 = low, 1 = high)."),
    XRT_FUNCTION("get_right_hand_is_active", getRightHandIsActive, "int", "Get the right hand tracking quality (0 = low, 1 = high)."),

    // Body tracking functions
    XRT_FUNCTION("is_body_data_available", isBodyDataAvailable, "bool", "Check if body tracking data is available."),
    XRT_FUNCTION("get_body_joints_pose", getBodyJointsPose, "list[list[float]]", "Get the body joints pose data (24 joints, 7 values each: x,y,z,qx,qy,qz,qw)."),
    XRT_FUNCTION("get_body_joints_velocity", getBodyJointsVelocity, "list[list[float]]", "Get the body joints velocity data (24 joints, 6 values each: vx,vy,vz,wx,wy,wz)."),
    XRT_FUNCTION("get_body_joints_acceleration", getBodyJointsAcceleration, "list[list[float]]", "Get the body joints acceleration data (24 joints, 6 values each: ax,ay,az,wax,way,waz)."),
    XRT_FUNCTION("get_body_joints_timestamp", getBodyJointsTimestamp, "list[int]", "Get the body joints IMU timestamp data (24 joints)."),
    XRT_FUNCTION("get_body_timestamp_ns", getBodyTimeStampNs, "int", "Get the body data timestamp in nanoseconds."),

    // Motion tracker functions
    XRT_FUNCTION("num_motion_data_available", numMotionDataAvailable, "int", "Check if motion tracker data is available."),
    XRT_FUNCTION("get_motion_tracker_pose", getMotionTrackerPose, "list[list[float]]", "Get the motion tracker pose data (3 trackers, 7 values each: x,y,z,qx,qy,qz,qw)."),
    XRT_FUNCTION("get_motion_tracker_velocity", getMotionTrackerVelocity, "list[list[float]]", "Get the motion tracker velocity data (3 trackers, 6 values each: vx,vy,vz,wx,wy,wz)."),
    XRT_FUNCTION("get_motion_tracker_acceleration", getMotionTrackerAcceleration, "list[list[float]]", "Get the motion tracker acceleration data (3 trackers, 6 values each: ax,ay,az,wax,way,waz)."),
    XRT_FUNCTION("get_motion_tracker_serial_numbers", getMotionTrackerSerialNumbers, "list[str]", "Get the serial numbers of the motion trackers."),
    XRT_FUNCTION("get_motion_timestamp_ns", getMotionTimeStampNs, "int", "Get the motion data timestamp in nanoseconds."),

    // send json bytes functions
    {"device_control_json", pyDeviceControlJson, METH_VARARGS,
     "device_control_json(dev_id: str, json_str: str, /) -> int\n\nSend a JSON control command to a device"},
    {"send_bytes_to_device", pySendBytesToDevice, METH_VARARGS,
     "send_bytes_to_device(dev_id: str, data: bytes, /) -> int\n\nSend raw bytes to a device"},
    {nullptr, nullptr, 0, nullptr},
};

#undef XRT_FUNCTION

PyModuleDef kModule = {
    PyModuleDef_HEAD_INIT,
    "_core",
    "Python bindings for PXREARobot SDK.",
    -1,  // the SDK state above is process-global
    kMethods,
};

}  // namespace

PyMODINIT_FUNC PyInit__core() {
    return PyModule_Create(&kModule);
}

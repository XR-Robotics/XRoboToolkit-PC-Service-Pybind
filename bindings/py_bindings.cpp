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
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include "PXREARobotSDK.h"


using json = nlohmann::json;
namespace py = pybind11;

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

// Meta IOBT body tracking ("BodyMeta"), kept entirely separate from the PICO "Body" path above.
// Two reasons it cannot reuse it: the arrays there are fixed at 24 and would silently drop 46 of
// the 70 UpperBody joints, and Meta reports no velocity/acceleration at all (BodyJointLocation
// carries only LocationFlags and Pose, unlike PICO's IMU trackers).
//
// std::vector rather than std::array: the joint count is whatever the client sends, so adding
// joints upstream -- or switching to the 84-joint FullBody skeleton -- needs no change here.
std::vector<int> BodyMetaJointIds;                       // Meta joint id, see get_body_meta_joint_ids
std::vector<std::array<double, 7>> BodyMetaJointsPose;   // x,y,z,qx,qy,qz,qw per joint
std::vector<bool> BodyMetaPositionValid;                 // per joint, false when occluded
std::vector<bool> BodyMetaOrientationValid;              // per joint, fails independently of above
std::array<double, 7> BodyMetaTrackingSpace{0, 0, 0, 0, 0, 0, 1};  // tracking space in world frame
std::string BodyMetaJointSet;                            // "UpperBody" / "FullBody"
std::string BodyMetaFidelity;                            // "High" = camera-measured IOBT
std::string BodyMetaCalibration;                         // "Valid" once skeleton scale settles
double BodyMetaConfidence = 0.0;
int64_t BodyMetaTimeStampNs = 0;
bool BodyMetaDataAvailable = false;

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
std::mutex bodyMetaMutex;  // Mutex for Meta IOBT body tracking data
std::mutex motionMutex;



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
    switch (type)
    {
    case PXREAServerConnect:
        std::cout << "server connect\n" << std::endl;
        break;
    case PXREAServerDisconnect:
        std::cout << "server disconnect\n" << std::endl;
        break;
    case PXREADeviceFind:
        std::cout << "device found\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing\n" << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect\n" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson:
        auto& dsj = *((PXREADevStateJson*)userData);
        

        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(leftMutex);
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
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightMutex);
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
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        HeadsetPose = stringToPoseArray(headset["pose"].get<std::string>());
                    }
                }
                if (value.contains("timeStampNs")) {
                    std::lock_guard<std::mutex> lock(timestampMutex);
                    TimeStampNs = value["timeStampNs"].get<int64_t>();
                }
                if (value["Hand"].contains("leftHand")) {
                    auto& leftHand = value["Hand"]["leftHand"];
                    {
                        std::lock_guard<std::mutex> lock(leftHandMutex);
                        
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
                // Parse BodyMeta: Meta IOBT upper-body joints from the Quest client. Independent
                // of the "Body" branch above -- neither reads the other's key or storage.
                if (value.contains("BodyMeta")) {
                    auto& bodyMeta = value["BodyMeta"];
                    {
                        std::lock_guard<std::mutex> lock(bodyMetaMutex);

                        // Unlike PICO, the timestamp is at the top level of the packet, not inside
                        // the body object: all sources in a frame share one, which is what makes
                        // headset pose and joints alignable.
                        if (value.contains("timeStampNs")) {
                            BodyMetaTimeStampNs = value["timeStampNs"].get<int64_t>();
                        }

                        // The headset's mount detection gates tracking, so once it leaves the head
                        // (neck included) isActive goes 0 while "joints" still holds the last frame
                        // -- the client reuses its JSON objects and does not clear them. Reporting
                        // unavailable is the whole point of the flag; a stale frame read as live is
                        // worse than no frame, because nothing downstream can tell it is stale.
                        const bool isActive = bodyMeta.contains("isActive") &&
                                              bodyMeta["isActive"].get<int>() != 0;
                        if (!isActive) {
                            BodyMetaDataAvailable = false;
                        } else if (bodyMeta.contains("joints") && bodyMeta["joints"].is_array()) {
                            if (bodyMeta.contains("jointSet")) {
                                BodyMetaJointSet = bodyMeta["jointSet"].get<std::string>();
                            }
                            if (bodyMeta.contains("fidelity")) {
                                BodyMetaFidelity = bodyMeta["fidelity"].get<std::string>();
                            }
                            if (bodyMeta.contains("calib")) {
                                BodyMetaCalibration = bodyMeta["calib"].get<std::string>();
                            }
                            if (bodyMeta.contains("confidence")) {
                                BodyMetaConfidence = bodyMeta["confidence"].get<double>();
                            }
                            if (bodyMeta.contains("trackingSpace")) {
                                BodyMetaTrackingSpace = stringToPoseArray(
                                    bodyMeta["trackingSpace"].get<std::string>());
                            }

                            auto& joints = bodyMeta["joints"];
                            const size_t jointCount = joints.size();

                            // Resized, never truncated: the count is data, not a constant.
                            BodyMetaJointIds.resize(jointCount);
                            BodyMetaJointsPose.resize(jointCount);
                            BodyMetaPositionValid.resize(jointCount);
                            BodyMetaOrientationValid.resize(jointCount);

                            for (size_t i = 0; i < jointCount; i++) {
                                auto& joint = joints[i];

                                BodyMetaJointIds[i] = joint.contains("id")
                                    ? joint["id"].get<int>() : static_cast<int>(i);

                                if (joint.contains("p")) {
                                    BodyMetaJointsPose[i] =
                                        stringToPoseArray(joint["p"].get<std::string>());
                                }

                                // Occluded joints keep their last pose, so validity has to travel
                                // with the data for consumers to know what to trust.
                                BodyMetaPositionValid[i] =
                                    joint.contains("v") && joint["v"].get<int>() != 0;
                                BodyMetaOrientationValid[i] =
                                    joint.contains("vr") && joint["vr"].get<int>() != 0;
                            }

                            BodyMetaDataAvailable = true;
                        }
                    }
                }
                //parse individual tracker data
                if (value.contains("Motion")) {
                    auto& motion = value["Motion"];
                    {
                        std::lock_guard<std::mutex> lock(motionMutex);
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

void init() {
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        throw std::runtime_error("PXREAInit failed");
    }
}

void deinit() {
    PXREADeinit();
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

// Meta IOBT body tracking functions. Named get_body_meta_* alongside the get_body_* above: the
// two are parallel, independent sources, and neither name is taken from the other.
//
// There is deliberately no get_body_meta_joints_velocity/_acceleration/_timestamp counterpart:
// Meta's BodyJointLocation has no such fields, and returning zeros under a familiar name would be
// worse than the function not existing.
bool isBodyMetaAvailable() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaDataAvailable;
}

std::vector<std::array<double, 7>> getBodyMetaJointsPose() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaJointsPose;
}

std::vector<int> getBodyMetaJointIds() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaJointIds;
}

std::vector<bool> getBodyMetaPositionValid() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaPositionValid;
}

std::vector<bool> getBodyMetaOrientationValid() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaOrientationValid;
}

std::array<double, 7> getBodyMetaTrackingSpace() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaTrackingSpace;
}

int64_t getBodyMetaTimeStampNs() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    return BodyMetaTimeStampNs;
}

// Grouped into one dict rather than six getters: these are read together when deciding whether a
// frame is trustworthy, and each would otherwise take its own lock.
py::dict getBodyMetaInfo() {
    std::lock_guard<std::mutex> lock(bodyMetaMutex);
    py::dict info;
    info["available"] = BodyMetaDataAvailable;
    info["count"] = BodyMetaJointsPose.size();
    info["jointSet"] = BodyMetaJointSet;
    info["fidelity"] = BodyMetaFidelity;
    info["calib"] = BodyMetaCalibration;
    info["confidence"] = BodyMetaConfidence;
    info["timeStampNs"] = BodyMetaTimeStampNs;
    return info;
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

    // Meta IOBT body tracking (Quest). Parallel to the get_body_* group above and independent of
    // it: these read the "BodyMeta" key, those read "Body". Joint count is not fixed (70 for the
    // UpperBody skeleton) and every joint is camera-measured.
    m.def("is_body_meta_available", &isBodyMetaAvailable,
          "Check if Meta IOBT body data is available. Call this before reading anything else: it "
          "goes False whenever the headset leaves the head, neck included, and the joint arrays "
          "then hold stale values that look perfectly valid.");
    m.def("get_body_meta_joints_pose", &getBodyMetaJointsPose,
          "Get Meta IOBT joint poses, (N, 7): x,y,z,qx,qy,qz,qw. Coordinates are TRACKING-SPACE "
          "local, so they describe posture only -- combine with get_body_meta_tracking_space() to "
          "recover where the operator is standing.");
    m.def("get_body_meta_joint_ids", &getBodyMetaJointIds,
          "Get Meta IOBT joint ids, (N,). Index into Meta's BodyJointId enum; read together with "
          "get_body_meta_info()['jointSet'], since UpperBody and FullBody number joints "
          "differently.");
    m.def("get_body_meta_position_valid", &getBodyMetaPositionValid,
          "Get per-joint position validity, (N,). False under occlusion, where the joint keeps its "
          "last position.");
    m.def("get_body_meta_orientation_valid", &getBodyMetaOrientationValid,
          "Get per-joint orientation validity, (N,). Fails independently of position validity.");
    m.def("get_body_meta_tracking_space", &getBodyMetaTrackingSpace,
          "Get the tracking space pose in world coordinates, (7,). Required to reconstruct root "
          "locomotion: joint coordinates are local to this space.");
    m.def("get_body_meta_timestamp_ns", &getBodyMetaTimeStampNs,
          "Get the Meta IOBT data timestamp in nanoseconds. Shared with all other sources in the "
          "same packet, so poses across sources are directly comparable.");
    m.def("get_body_meta_info", &getBodyMetaInfo,
          "Get data-quality metadata: available, count, jointSet, fidelity, calib, confidence, "
          "timeStampNs. Gate on calib == 'Valid' (skeleton scale is still settling before that) "
          "and fidelity == 'High' (Low is IK inference, not camera-measured).");

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
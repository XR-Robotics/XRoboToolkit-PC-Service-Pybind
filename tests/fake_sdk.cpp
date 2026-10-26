// Test-only replacement for the SDK ABI. No socket or hardware is opened.
#include <cstring>
#include <string>
#include "PXREARobotSDK.h"

static pfPXREAClientCallback callback = nullptr;
static void* callbackContext = nullptr;
static int initResult = 0;
static int initCalls = 0;
static int closeCalls = 0;

extern "C" int PXREAInit(void* context, pfPXREAClientCallback cb, unsigned) {
    ++initCalls;
    callbackContext = context;
    callback = cb;
    return initResult;
}

extern "C" int PXREADeinit() {
    ++closeCalls;
    // Simulate an in-flight callback during close: it must be ignored.
    if (callback) {
        callback(callbackContext, PXREAServerDisconnect, 0, nullptr);
    }
    return 0;
}

extern "C" int PXREADeviceControlJson(const char*, const char*) { return 0; }
extern "C" int PXREASendBytesToDevice(const char*, const char*, unsigned) { return 0; }

extern "C" int xrt_test_emit_state(const char* device, const char* payload) {
    if (!callback || std::strlen(device) >= 32 || std::strlen(payload) >= 16352) {
        return -1;
    }
    PXREADevStateJson state{};
    std::strcpy(state.devID, device);
    std::strcpy(state.stateJson, payload);
    callback(callbackContext, PXREADeviceStateJson, 0, &state);
    return 0;
}

extern "C" void xrt_test_emit_event(int type, const char* device) {
    if (callback) {
        callback(callbackContext, static_cast<PXREAClientCallbackType>(type), 0,
                 const_cast<char*>(device));
    }
}

extern "C" void xrt_test_init_result(int result) { initResult = result; }
extern "C" int xrt_test_init_calls() { return initCalls; }
extern "C" int xrt_test_close_calls() { return closeCalls; }

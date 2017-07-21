//
// Created by Tucker Haydon on 6/23/17.
//

#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/usb/_UsbStreamParams.h>
#include <iostream>
#include <functional>
#include <pylon/PylonImage.h>
#include <map>
#include <vector>
#include <memory>
#include <future>
#include <utility>
#include <sys/time.h>

#include "CallbackFunction.h"

class CameraManager {

public:
    CameraManager(const std::map<std::string, std::string> &params);
    ~CameraManager();

    void loop();
    void setFrameCallbackFunction(const CallbackFunction &cb);

private:
    Pylon::CBaslerUsbInstantCamera *camera;
    std::map<std::string, std::string> params;
    void initParameters();
    CallbackFunction cb;

};

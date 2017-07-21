//
// Created by tuckerhaydon on 7/10/17.
//

#pragma once

#include <pylon/PylonIncludes.h>
#include <functional>
#include <memory>

typedef std::function<void(std::unique_ptr<Pylon::CPylonImage> frame)> CallbackFunction;

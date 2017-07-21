//
// Created by tuckerhaydon on 7/7/17.
//

#pragma once
#include "FramePublisher.h"
#include "CameraManager.h"
#include "CallbackFunction.h"

#include <iostream>
#include <thread>
#include <vector>
#include <pylon/PylonIncludes.h>
#include <pylon/PylonImage.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <future>
#include <memory>

FramePublisher *framePublisher;
CameraManager *cameraManager;

void frameCallbackFunction(std::unique_ptr<Pylon::CPylonImage> frame);
void publisherLoop(FramePublisher *pub);
void cameraLoop(CameraManager *cm);
int main(int argc, char **argv);


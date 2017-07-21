//
// Created by Tucker Haydon on 6/23/17.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <memory>

#include "dart_image_streamer/RequestRawImages.h"

class FramePublisher {
public:
    FramePublisher(int argc, char** argv);
    ~FramePublisher();

    void publish(std::unique_ptr<sensor_msgs::Image> frame);
    void buffer(std::unique_ptr<sensor_msgs::Image> frame);
    void bufferAndPublish(std::unique_ptr<sensor_msgs::Image> frame);
    bool serverHandler(dart_image_streamer::RequestRawImages::Request &req, dart_image_streamer::RequestRawImages::Response &res);
    void loop();

    std::map<std::string, std::string> getCameraParams();
    std::atomic<bool> busy{false};

private:
    std::map<std::string, std::string> cameraParams;
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it;
    ros::ServiceServer service;
    image_transport::Publisher pub;
    std::vector<std::unique_ptr<sensor_msgs::Image>> frameBuffer;
    std::mutex bufferMtx;

    std::atomic<bool> bufferFrames{false};


};

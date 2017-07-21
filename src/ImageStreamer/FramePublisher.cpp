//
// Created by Tucker Haydon on 6/23/17.
//

#include "../include/FramePublisher.h"

using namespace std;
using namespace ros;

bool FramePublisher::serverHandler(dart_image_streamer::RequestRawImages::Request &req, dart_image_streamer::RequestRawImages::Response &res) {
    bufferFrames = true;
    int numFrames = req.numImages;
    int bufferSize = 0;

     do {
         {
             std::lock_guard<std::mutex> lock(bufferMtx);
             bufferSize = frameBuffer.size();
         }
         usleep(10000);
    } while(bufferSize < numFrames);

    bufferFrames = false;

    for(auto& framePtr: frameBuffer) {
        res.images.push_back(*framePtr);
    }

    frameBuffer.clear();

    return true;
}

FramePublisher::FramePublisher(int argc, char** argv) {
    init(argc, argv, "camera_publisher");
    this->nh = new NodeHandle("~");
    this->it = new image_transport::ImageTransport(*(this->nh));
    this->pub = it->advertise("raw_image", 1);
    this->service = nh->advertiseService("RequestRawImages", &FramePublisher::serverHandler, this);

    // Load the camera parameters
    if (!this->nh->getParam("camera/params", this->cameraParams)) {
        ROS_WARN("No camera params found!");
        exit(1);
    }

}

FramePublisher::~FramePublisher() {
    delete this->it;
    delete this->nh;
}

void FramePublisher::loop() {
    ros::Rate r(10);
    while(this->nh->ok()) {
        r.sleep();
        ros::spinOnce();
    }
}

map<string, string> FramePublisher::getCameraParams() {
    return this->cameraParams;
}

void FramePublisher::publish(unique_ptr<sensor_msgs::Image> frame) {
    this->busy = true;
    this->pub.publish(*frame);
    this->busy = false;
}

void FramePublisher::buffer(unique_ptr<sensor_msgs::Image> frame) {
    if(bufferFrames) {
        std::lock_guard<std::mutex> lock(bufferMtx);
        frameBuffer.push_back(std::move(frame));
    }
}

void FramePublisher::bufferAndPublish(unique_ptr<sensor_msgs::Image> frame) {
    // publish(frame);
    // buffer(frame);
}

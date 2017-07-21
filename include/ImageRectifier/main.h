#include <iostream>
#include <cstdlib>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera_streaming_pipeline/RectifyImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv);
void readCameraParams(const std::string &cameraParamsFileName);
bool serverHandler(camera_streaming_pipeline::RectifyImage::Request &req, camera_streaming_pipeline::RectifyImage::Response &res);

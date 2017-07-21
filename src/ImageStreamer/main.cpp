//
// Created by tuckerhaydon on 7/7/17.
//

#include "../../include/ImageStreamer/main.h"

using namespace std;
using namespace Pylon;
using namespace ros;

void frameCallbackFunction(unique_ptr<CPylonImage> frame) {

    if(framePublisher->busy) {
        return;
    }

    uint16_t width = frame->GetWidth();
    uint16_t height = frame->GetHeight();
    uint8_t *buffer = (uint8_t *)frame->GetBuffer();

    static int seq = 0;

    vector<uint8_t> data;
    data.assign(buffer, buffer + (width * height *3));

    unique_ptr<sensor_msgs::Image> im = std::make_unique<sensor_msgs::Image>();
    im->header.seq = seq++;
    im->header.stamp = ros::Time::now();
    im->header.frame_id = "";
    im->height=height;
    im->width=width;
    im->encoding=sensor_msgs::image_encodings::RGB8;
    im->is_bigendian=true;
    im->step=width * 1 * 3;
    im->data=data;

    framePublisher->buffer(std::move(im));
}

void publisherLoop(FramePublisher *pub) {
    pub->loop();
}

void cameraLoop(CameraManager *cm) {
    cm->loop();
}


int main(int argc, char **argv) {
    framePublisher = new FramePublisher(argc, argv);
    cameraManager = new CameraManager(framePublisher->getCameraParams());

    cameraManager->setFrameCallbackFunction(frameCallbackFunction);

    future<void> ret1 = async(launch::async, publisherLoop, framePublisher);
    future<void> ret2 = async(launch::async, cameraLoop, cameraManager);

    ret1.get();
    ret2.get();

    delete framePublisher;
    delete cameraManager;

    return 0;
}

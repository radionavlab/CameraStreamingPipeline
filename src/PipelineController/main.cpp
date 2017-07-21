#include "../../include/PipelineController/main.h"

using namespace std;
using namespace ros;


int main(int argc, char **argv) {

    init(argc, argv, "pipeline_controller");
    NodeHandle nh{"~"};

    ServiceClient rawImagesClient = nh.serviceClient<camera_streaming_pipeline::RequestRawImages>("/image_streamer/RequestRawImages");
    // ServiceClient bestImageClient = nh.serviceClient<image_pipeline_controller::RequestBestImage>("RequestRawImages");
    ServiceClient rectifiedImageClient = nh.serviceClient<camera_streaming_pipeline::RectifyImage>("/image_rectifier/RectifyImage");

    camera_streaming_pipeline::RequestRawImages rawImagesSrv;
    // image_pipeline_controller::RequestBestImage bestImageSrv;
    camera_streaming_pipeline::RectifyImage rectifiedImageSrv;

    image_transport::ImageTransport it{nh};
    image_transport::Publisher pub = it.advertise("undistorted_images", 1);


    Rate r(10);
    while(nh.ok()) {

        rawImagesSrv.request.numImages = 1;
        rawImagesClient.call(rawImagesSrv);

        for(auto &im: rawImagesSrv.response.images) {
            rectifiedImageSrv.request.distortedImage = im;
            rectifiedImageClient.call(rectifiedImageSrv);
            pub.publish(rectifiedImageSrv.response.rectifiedImage);
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

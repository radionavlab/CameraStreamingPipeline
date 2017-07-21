#include "../../include/PipelineController/main.h"

using namespace std;
using namespace ros;


int main(int argc, char **argv) {

    init(argc, argv, "image_pipeline_controller");
    NodeHandle nh{"~"};

    ServiceClient rawImagesClient = nh.serviceClient<image_pipeline_controller::RequestRawImages>("/camera_publisher/RequestRawImages");
    // ServiceClient bestImageClient = nh.serviceClient<image_pipeline_controller::RequestBestImage>("RequestRawImages");
    ServiceClient rectifiedImageClient = nh.serviceClient<image_pipeline_controller::RectifyImage>("/image_rectifier/RectifyImage");

    camera_streaming_pipeline::RequestRawImages rawImagesSrv;
    // image_pipeline_controller::RequestBestImage bestImageSrv;
    camera_streaming_pipeline::RectifyImage rectifiedImageSrv;

    Rate r(10);
    while(nh.ok()) {

        rawImagesSrv.request.numImages = 1;
        /*
        if(rawImagesClient.call(rawImagesSrv)) {
            cout << rawImagesSrv.response.images.size() << endl;
        } else {
            cout << "Failed." << endl;
        }
        */

        rawImagesClient.call(rawImagesSrv);

        for(auto &im: rawImagesSrv.response.images) {
            rectifiedImageSrv.request.distortedImage = im;
            rectifiedImageClient.call(rectifiedImageSrv);
        }


        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

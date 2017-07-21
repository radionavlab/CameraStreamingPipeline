
#include "../../include/ImageRectifier/main.h"

using namespace cv;
using namespace std;
using namespace ros;

string cameraParamsFileName;
Mat cameraMatrix;
Mat distortionCoefficients;

bool serverHandler(camera_streaming_pipeline::RectifyImage::Request &req, camera_streaming_pipeline::RectifyImage::Response &res) {
    sensor_msgs::Image inputImage = req.distortedImage;
    Mat inputMat = cv_bridge::toCvCopy(inputImage, "bgr8")->image;

    Mat outputMat;
    undistort(inputMat, outputMat, cameraMatrix, distortionCoefficients);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image outputImage;
    
    img_bridge = cv_bridge::CvImage(inputImage.header, sensor_msgs::image_encodings::BGR8, outputMat);
    img_bridge.toImageMsg(outputImage);

	res.rectifiedImage = outputImage;

    return true;
}

void readCameraParams(const string &cameraParamsFileName) {
    cout << cameraParamsFileName << endl;
    FileStorage fs;
    fs.open(cameraParamsFileName.c_str(), FileStorage::READ);
    if( !fs.isOpened() ){
        cerr << " Fail to open camera parameter file." << endl;
        exit(EXIT_FAILURE);
    }

    // Get camera parameters
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoefficients; 

    // Print out the camera parameters
    cout << "\n -- Camera parameters -- " << endl;
    cout << "\n CameraMatrix = " << endl << " " << cameraMatrix << endl << endl;
    cout << " Distortion coefficients = " << endl << " " << distortionCoefficients << endl << endl;

    fs.release();
}

int main(int argc, char** argv) {
    init(argc, argv, "image_rectifier");
    NodeHandle nh{"~"};

    if (!nh.getParam("/image_rectifier/distortion_parameters", cameraParamsFileName)) {
        ROS_WARN("No camera params file specified! Exiting!");
        exit(1);
    }

    readCameraParams(cameraParamsFileName);

    ServiceServer service = nh.advertiseService("RectifyImage", serverHandler);

    Rate r(10);
    while(nh.ok()) {
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

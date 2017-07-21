
#include "../../include/ImageRectifier/main.h"

using namespace cv;
using namespace std;
using namespace ros;

string cameraParamsFileName;
Mat cameraMatrix;
Mat distortionCoefficients;

bool serverHandler(camera_streaming_pipeline::RectifyImage::Request &req, camera_streaming_pipeline::RectifyImage::Response &res) {
    sensor_msgs::Image input = req.distortedImage;
    Mat inputImage = cv_bridge::toCvCopy(input, "bgr8")->image;

    if( !inputImage.data ){
        cout << "Could not load image" << endl;
        exit(EXIT_FAILURE);
    }

    Mat outputImage;
    undistort(inputImage, outputImage, cameraMatrix, distortionCoefficients);
    
    imshow("img", outputImage);
    waitKey(0);

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

    if (!nh.getParam("/image_rectifier/params/camera_params", cameraParamsFileName)) {
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

#include "../../include/ImageStreamer/CameraManager.h"

using namespace Pylon;
using namespace std;
using namespace GenApi;
using namespace Basler_UsbCameraParams;

void time() {
    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    cout << currentTime - lastTime << endl;
    lastTime = currentTime;
}

CameraManager::CameraManager(const std::map<std::string, std::string> &params) {
    this->params = params;
    PylonInitialize();
    try{
        // Create USB Camera with first camera device found
        this->camera = new CBaslerUsbInstantCamera( CTlFactory::GetInstance().CreateFirstDevice());

        // Initialize the camera parameters
        this->initParameters();

    } catch(const GenericException &e) {
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exit(1);
    }
}

CameraManager::~CameraManager() {
    delete camera;
    PylonTerminate();
}

void CameraManager::initParameters() {
    try{
        // Open the camera for accessing the parameters.
        this->camera->Open();

        /* Set parameters */
        this->camera->Width.FromString(gcstring(params.at("Width").c_str()));
        this->camera->Height.FromString(gcstring(params.at("Height").c_str()));
        this->camera->PixelFormat.FromString(gcstring(params.at("PixelFormat").c_str()));

        this->camera->GetStreamGrabberParams().MaxNumBuffer.SetValue(20);
        this->camera->GetStreamGrabberParams().NumMaxQueuedUrbs.SetValue(5);
        this->camera->GetStreamGrabberParams().MaxTransferSize.SetValue(4194304);
        this->camera->GetStreamGrabberParams().MaxBufferSize.SetValue(4194304);

        /*
        // Limit the throughput
        this->camera->DeviceLinkSelector.SetValue(0);
        this->camera->DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitMode_On);
        this->camera->DeviceLinkThroughputLimit.SetValue(32000000);
        */
      
        /*
        this->camera->AutoExposureTimeUpperLimit.FromString(gcstring(params.at("AutoExposureTimeUpperLimit").c_str()));
        this->camera->BalanceWhiteAuto.FromString(gcstring(params.at("BalanceWhiteAuto").c_str()));
        this->camera->GainAuto.FromString(gcstring(params.at("GainAuto").c_str()));
        this->camera->LightSourcePreset.FromString(gcstring(params.at("LightSourcePreset").c_str()));
        */


        this->camera->Close();

    } catch(const GenericException &e) {
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exit(1);
    }
}

void CameraManager::loop() {
    try {
        this->camera->StartGrabbing();
        CGrabResultPtr ptrGrabResult;

        while (this->camera->IsGrabbing()) {
            this->camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            if (ptrGrabResult->GrabSucceeded()) {
                time();
                unique_ptr<CPylonImage> frame = make_unique<CPylonImage>();
                frame->AttachGrabResultBuffer(ptrGrabResult);
                thread(this->cb, move(frame)).detach();
            }
            else {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }
    }
    catch (const GenericException &e) {
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
    }

}

void CameraManager::setFrameCallbackFunction(const CallbackFunction &cb) {
    this->cb = cb;
}


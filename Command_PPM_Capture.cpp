//=============================================================================
// Copyright (c) 2001-2018 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
 *  @example SaveToAvi.cpp
 *
 *  @brief SaveToAvi.cpp shows how to create an video from a vector of images.
 *  It relies on information provided in the Enumeration, Acquisition, and
 *  NodeMapInfo examples.
 *
 *  This example introduces the SpinVideo class, which is used to quickly and
 *  easily create various types of video files. It demonstrates the creation of
 *  three types: uncompressed, MJPG, and H264.
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>

//NBL: timer() includes
#include <ctime>
#include <cstdlib>

//NBL: kbhit() includes
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "SpinVideo.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace Spinnaker::Video;
using namespace std;

// Use the following enum and global constant to select the type of video
// file to be created and saved.
enum videoType
{
    UNCOMPRESSED,
    MJPG,
    H264
};

const videoType chosenVideoType = UNCOMPRESSED;

//NBL: Look for a keyboard hit.  This will be put to use to start and stop video recording.
//Credit to: https://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html
int kbhit(int &ch)
{
  struct termios oldt, newt;
  //int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

/*
//NBL: Hang the program for a specified number of seconds.
//May need to simultaneously record, and not just delay....
void timer(double seconds_to_delay, bool &is_capturing){
    clock_t start_time = clock(); //Start timer
    double seconds_passed;
    bool flag = true;
    while(flag)
    {
        seconds_passed = (clock() - start_time) / CLOCKS_PER_SEC;
        if(seconds_passed >= seconds_to_delay)
        {
            flag = false;
        }
    }
    is_capturing = false;
}*/

void start_timer(clock_t & start_time){
    start_time = clock();
}

void check_timer(clock_t start_time, double seconds_to_capture, bool & limit_reached){
    double seconds_passed;
    
    seconds_passed = (clock() - start_time) / CLOCKS_PER_SEC;
    if(seconds_passed >= seconds_to_capture){
        limit_reached = false;
    }
    
    std::cout << "start_time: " << start_time << " seconds_passed: " << seconds_passed << " seconds_to_capture: " << seconds_to_capture << endl;
    
}

// This function configures a custom exposure time. Automatic exposure is turned 
// off in order to allow for the customization, and then the custom setting is 
// applied.
int ConfigureExposure(INodeMap & nodeMap)
{
        int result = 0;
        cout << endl << endl << "*** CONFIGURING EXPOSURE ***" << endl << endl;
        try
        {       
                //
                // Turn off automatic exposure mode
                //
                // *** NOTES ***
                // Automatic exposure prevents the manual configuration of exposure 
                // time and needs to be turned off.
                //
                // *** LATER ***
                // Exposure time can be set automatically or manually as needed. This
                // example turns automatic exposure off to set it manually and back
                // on in order to return the camera to its default state.
                //
                CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
                if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
                {
                        cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
                        return -1;
                }
                
                CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
                if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
                {
                        cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
                        return -1;
                }
                
                ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
                
                cout << "Automatic exposure disabled..." << endl;
                //
                // Set exposure time manually; exposure time recorded in microseconds
                //
                // *** NOTES ***
                // The node is checked for availability and writability prior to the 
                // setting of the node. Further, it is ensured that the desired exposure 
                // time does not exceed the maximum. Exposure time is counted in 
                // microseconds. This information can be found out either by 
                // retrieving the unit with the GetUnit() method or by checking SpinView.
                // 
                CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
                if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
                {
                        cout << "Unable to set exposure time. Aborting..." << endl << endl;
                        return -1;
                }
                
                // Ensure desired exposure time does not exceed the maximum
                const double exposureTimeMax = ptrExposureTime->GetMax();
                double exposureTimeToSet = 9000.0;//2000000.0;
                if (exposureTimeToSet > exposureTimeMax)
                {
                        exposureTimeToSet = exposureTimeMax;
                }
                
                ptrExposureTime->SetValue(exposureTimeToSet);
                
                cout << "Exposure time set to " << exposureTimeToSet << " us..." << endl << endl;
        }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}

// This function returns the camera to its default state by re-enabling automatic
// exposure.
int ResetExposure(INodeMap & nodeMap)
{
        int result = 0;
        try
        {
                // 
                // Turn automatic exposure back on
                //
                // *** NOTES ***
                // Automatic exposure is turned on in order to return the camera to its 
                // default state.
                //
                CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
                if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
                {
                        cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
                        return -1;
                }
        
                CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
                if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
                {
                        cout << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
                        return -1;
                }
                ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
                cout << "Automatic exposure enabled..." << endl << endl;
        }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}


// This function prepares, saves, and cleans up an video from a vector of images.
int SaveVectorToVideo(INodeMap & nodeMap, INodeMap & nodeMapTLDevice, vector<ImagePtr> & images)
{
    int result = 0;

    cout << endl << endl << "*** CREATING VIDEO ***" << endl << endl;

    try
    {
        // Retrieve device serial number for filename
        string deviceSerialNumber = "";

        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
        }

        //
        // Get the current frame rate; acquisition frame rate recorded in hertz
        //
        // *** NOTES ***
        // The video frame rate can be set to anything; however, in order to
        // have videos play in real-time, the acquisition frame rate can be
        // retrieved from the camera.
        //
        CFloatPtr ptrAcquisitionFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
        if (!IsAvailable(ptrAcquisitionFrameRate) || !IsReadable(ptrAcquisitionFrameRate))
        {
            cout << "Unable to retrieve frame rate. Aborting..." << endl << endl;
            return -1;
        }

        float frameRateToSet = static_cast<float>(ptrAcquisitionFrameRate->GetValue());

        cout << "Frame rate to be set to " << frameRateToSet << "..." << endl;

        //
        // Create a unique filename
        //
        // *** NOTES ***
        // This example creates filenames according to the type of video
        // being created. Notice that '.avi' does not need to be appended to the
        // name of the file. This is because the SpinVideo object takes care
        // of the file extension automatically.
        //
        string videoFilename;

        switch (chosenVideoType)
        {
        case UNCOMPRESSED:
            videoFilename = "Avi-Raw";
            if (deviceSerialNumber != "")
            {
                videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
            }

            break;

        case MJPG:
            videoFilename = "SaveToAvi-MJPG";
            if (deviceSerialNumber != "")
            {
                videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
            }

            break;

        case H264:
            videoFilename = "SaveToAvi-H264";
            if (deviceSerialNumber != "")
            {
                videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
            }
        }

        //
        // Select option and open video file type
        //
        // *** NOTES ***
        // Depending on the file type, a number of settings need to be set in
        // an object called an option. An uncompressed option only needs to
        // have the video frame rate set whereas videos with MJPG or H264
        // compressions should have more values set.
        //
        // Once the desired option object is configured, open the video file
        // with the option in order to create the video file.
        //
        // *** LATER ***
        // Once all images have been added, it is important to close the file -
        // this is similar to many other standard file streams.
        //
        SpinVideo video;

        // Set maximum video file size to 2GB.
        // A new video file is generated when 2GB
        // limit is reached. Setting maximum file
        // size to 0 indicates no limit.
        const unsigned int k_videoFileSize = 0;//2048;

        video.SetMaximumFileSize(k_videoFileSize);

        if (chosenVideoType == UNCOMPRESSED)
        {
            Video::AVIOption option;

            option.frameRate = frameRateToSet;

            video.Open(videoFilename.c_str(), option);
        }
        else if (chosenVideoType == MJPG)
        {
            Video::MJPGOption option;

            option.frameRate = frameRateToSet;
            option.quality = 75;

            video.Open(videoFilename.c_str(), option);
        }
        else if (chosenVideoType == H264)
        {
            Video::H264Option option;

            option.frameRate = frameRateToSet;
            option.bitrate = 1000000;
            option.height = static_cast<unsigned int>(images[0]->GetHeight());
            option.width = static_cast<unsigned int>(images[0]->GetWidth());

            video.Open(videoFilename.c_str(), option);
        }

        //
        // Construct and save video
        //
        // *** NOTES ***
        // Although the video file has been opened, images must be individually
        // appended in order to construct the video.
        //
        cout << "Appending " << images.size() << " images to video file: " << videoFilename << ".avi... " << endl << endl;

        for (unsigned int imageCnt = 0; imageCnt < images.size(); imageCnt++)
        {
            video.Append(images[imageCnt]);

            cout << "\tAppended image " << imageCnt << "..." << endl;
        }

        //
        // Close video file
        //
        // *** NOTES ***
        // Once all images have been appended, it is important to close the
        // video file. Notice that once an video file has been closed, no more
        // images can be added.
        //
        video.Close();

        cout << endl << "Video saved at " << videoFilename << ".avi" << endl << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap)
{
    int result = 0;

    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// This function acquires and saves 10 images from a device; please see
// Acquisition example for more in-depth comments on acquiring images.
int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, vector<ImagePtr> & images)
{
    int result = 0;
    bool is_capturing = false;
    int key = '~';
    string imageFilename = "images/image-";
    cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    try
    {
        // Set acquisition mode to continuous
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl << endl;
            return -1;
        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "Acquisition mode set to continuous..." << endl;

        //NBL: There is at least one camera, so look for a command to start recording.
        //record as long as you don't get a command to stop recording.  Commands are entered
        //in the command line of a terminal.  Usr will be prompted with appropriate information.
        std::cout << "Press SPACE to start capture...." << endl;
        //getchar();
        while(key != ' ' && is_capturing == false){
            //puts("Press a key!");       
            kbhit(key);
            if(key == ' '){
                is_capturing = true;
            }
        }
        std::cout << "Begin capturing... press 'q' to stop..." << endl;
        getchar();
        pCam->BeginAcquisition();

        unsigned int imageCnt = 0;
        clock_t capture_start_time;
        double capture_run_time = 60;
        start_timer(capture_start_time);
        while(is_capturing == true){
        //while(key != 'q' && is_capturing == true){
            //NBL: <START ACQUIRE/>
            // Begin acquiring images
            
            cout << "Acquiring images..." << endl << endl;
                
            // Retrieve and convert images
            
            imageCnt++;
            
            // Retrieve the next received image
            //clock_t start_get, stop_get;
            //start_get = clock();
            ImagePtr pResultImage = pCam->GetNextImage();
            if(imageCnt % 5 == 0){    
                
                try
                {
                    if (pResultImage->IsIncomplete())
                    {
                        cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
                    }
                    else
                    {
                        cout << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
                        imageFilename = imageFilename + to_string(imageCnt) + ".ppm";
                        cout << "Grabbed image at " << __TIME__ << " to save to " << imageFilename.c_str() << endl;
                        
                        pResultImage->Save(imageFilename.c_str(), PPM);
                        //cout << "Saved image file " << imageFilename << endl;
                        
                        imageFilename = "images/image-";

                        // Deep copy image into image vector
                        //images.push_back(pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR));//PixelFormat_Mono8, HQ_LINEAR));
                    }
                    // Release image
                    pResultImage->Release();
                }
                catch (Spinnaker::Exception &e)
                {
                    cout << "Error: " << e.what() << endl;
                    result = -1;
                }
                //stop_get = clock();
                //double capture_duration = stop_get - start_get;
                //std::cout << "duration for GetNextImage() = " << capture_duration/CLOCKS_PER_SEC <<endl;
                check_timer(capture_start_time, capture_run_time, is_capturing);
            }
            /*
            //NBL: </END ACQUIRE> 
            //puts("Capturing!");       
            kbhit(key);
            printf("You pressed '%c'!\n", key);
            if(key == 'q'){
                timer(5, is_capturing);
                
            }*/
        }
        std::cout << "Done capturing.  Please come again." << endl << endl;
        
        // End acquisition
        pCam->EndAcquisition();
        
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
    int result = 0;
    int err = 0;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();

 /*       // Configure exposure
        err = ConfigureExposure(nodeMap);
        if (err < 0)
        {
                return err;
        }        
   */     
        // Acquire images and save into vector
        vector<ImagePtr> images;

        err = AcquireImages(pCam, nodeMap, images);
        if (err < 0)
        {
            return err;
        }

        // Reset exposure
     //   result = result | ResetExposure(nodeMap);
        
        // Save vector of images to video
        //result = result | SaveVectorToVideo(nodeMap, nodeMapTLDevice, images);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}


// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
    int result = 0;
    
    
    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build << endl << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Uh oh! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        cout << endl << "Running example for camera " << i << "..." << endl;

        result = result | RunSingleCamera(camList.GetByIndex(i));

        cout << "Camera " << i << " example complete..." << endl << endl;
    }


    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Recording Complete! Press Enter to exit..." << endl;
    getchar();

    return result;
}

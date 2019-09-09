#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

//http://wiki.ros.org/spinnaker_sdk_camera_driver
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <ctime>


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace std::chrono;

//NBL: Typing issue?
std_msgs::String record;
gcstring deviceSerialNumber [] = {};
string desiredSN = "Blackfly BFLY-PGE-50S5C";
string deviceModelNameS = "";
CameraList camList;
unsigned int imageCnt;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //NBL: This should write the message to a global variable that gets checked
    //in main()...at bottom of main, do camera work, put spinOnce() at the bottom of the while loop
    //see 23 Aug, 2019 notes for details.
    //std::cout << "heard.... " << std::boolalpha << msg->data << std::endl;
    record = *msg;//->data.c_str();
 
    //ROS_INFO("I heard: [%s]", record.data.c_str());//msg->data.c_str());
}

int deviceRegistration(CameraPtr pCam, int cameraIndx){
    int err = 0;

    try
    {
        cout << "pCam->GetTLDeviceNodeMap()..." << endl;
        //NBL: From here....
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        cout << "pCam->Init()...." << endl;
        // Initialize camera
        pCam->Init();
        cout << "pCam->GetNodeMap()...." << endl;
        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();
        cout << "nodeMap.GetNode('DeviceID')...." << endl;
        CStringPtr ptrStringSerial = nodeMap.GetNode("DeviceID");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
                deviceSerialNumber[cameraIndx] = ptrStringSerial->GetValue();
                std::cout << "Device serial number retrieved as " << deviceSerialNumber[cameraIndx] << std::endl;
        }
        std::cout << std::endl;
         
        //NBL: .... to here should be run ONCE for each camera.  THIS is 
        //where serial number acquisition should happen.
        
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        err = -1;
    }
}
/*
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
*/
// This function acquires and saves 10 images from a device; please see
// Acquisition example for more in-depth comments on acquiring images.
int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, int cameraIndx)
{
    int result = 0;
    if(deviceModelNameS == desiredSN){
            
        system_clock::time_point now = system_clock::now();
        time_t tt = system_clock::to_time_t(now);
        tm utc_tm = *gmtime(&tt);
        tm local_tm = *localtime(&tt);
        string dateTime = to_string(utc_tm.tm_year + 1900) + '-' + to_string(utc_tm.tm_mon + 1) + '-' + to_string(utc_tm.tm_mday) + '_' 
                        + to_string(utc_tm.tm_hour) + '-' + to_string(utc_tm.tm_min) + '-' + to_string(utc_tm.tm_sec);
     
        string imageFilename = deviceModelNameS + "_" + dateTime;//"home/documents/images/image-";
        //cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
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
            pCam->BeginAcquisition();
     
            //clock_t capture_start_time;
            //NBL: Make this a parameter/prompt.  Is this the point
            //at which ROS compatability should be integrated? 
            //double capture_run_time = 10;
            //start_timer(capture_start_time);
            //while(is_capturing == true){
            if(record.data == "1"){
                //NBL: <START ACQUIRE/>
                // Begin acquiring images
                
                //cout << "Acquiring images..." << endl << endl;
                    
                // Retrieve and convert images
                
                imageCnt++;
                
                // Retrieve the next received image
                ImagePtr pResultImage = pCam->GetNextImage();
                //if(imageCnt % 1 == 0){    
                    
                try
                {
                    if (pResultImage->IsIncomplete())
                    {
                        cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
                    }
                    else
                    {
                        //cout << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
                        imageFilename = imageFilename + ".ppm";//deviceSerialNumber[cameraIndx].c_str() + "-" + to_string(imageCnt) + ".ppm";
                        //cout << "Grabbed image at " << __TIME__ << " to save to " << imageFilename.c_str() << endl;
                        
                        pResultImage->Save(imageFilename.c_str(), PPM);
                        //NBL: reset filename... should edit to include camera name
                        imageFilename = deviceModelNameS  + "-" + dateTime;
                    }
                    // Release image
                    pResultImage->Release();
                }
                catch (Spinnaker::Exception &e)
                {
                    cout << "Error: " << e.what() << endl;
                    result = -1;
                }
                //check_timer(capture_start_time, capture_run_time, is_capturing);
            }
            //}
            //std::cout << "Done capturing.  Please come again." << endl << endl;
            
            // End acquisition
            pCam->EndAcquisition();
            
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "Error: " << e.what() << endl;
            result = -1;
        }
    }
    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam, int cameraIndx)
{
    int result = 0;
    int err = 0;

    try
    {
        //NBL: From here....
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        //result = PrintDeviceInfo(nodeMapTLDevice);

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
        //NBL: .... to here should be run ONCE for each camera.  THIS is 
        //where serial number acquisition should happen.
        err = AcquireImages(pCam, nodeMap, cameraIndx);
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

int run_camera(CameraList &camList, unsigned int numCameras){
    int result;
    
    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        //cout << endl << "Running example for camera " << i << "..." << endl;

        result = result | RunSingleCamera(camList.GetByIndex(i), i);

        //cout << "Camera " << i << " example complete..." << endl << endl;
    }
    
}


int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "bool_sub");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    ros::Subscriber sub = n.subscribe("record", 100, chatterCallback);
    imageCnt = 0;
    ros::Rate loop_rate(10);

    //NBL: Camera initialization
    unsigned int numCameras = 0;   
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    
    while(desiredSN != deviceModelNameS){

        // Print out current library version   
        const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();

        cout << "while desiredSN != deviceModelNameS.... system->GetCameras()" << endl;
        // Retrieve list of cameras from the system
        camList = system->GetCameras();
        cout << "numCameras = camList.GetSize();" << endl;
        numCameras = camList.GetSize();
    
        cout << "Number of cameras detected: " << numCameras << endl << endl;

        // Prompt to check connections if there are no cameras
        if (numCameras == 0)
        {
            cout << "camList.Clear();" << endl;
            // Clear camera list before releasing system
            camList.Clear();
            cout << "system->ReleaseInstance();" << endl;
            // Release system
            system->ReleaseInstance();

            cout << "I am not finding any cameras connected.  Please press ENTER, and I will recheck in 5 seconds." << endl;
            //cout << "Uh oh! Press Enter to exit..." << endl;
            getchar();
            ros::Duration(5.0).sleep(); 
            cout << "numCameras = " << numCameras << endl;
            //return -1;
        }
        // Retrieve singleton reference to system object
        system = System::GetInstance();
        
        //cout << "deviceRegistration () for camera... ";
        CameraPtr pCam;
        string deviceSNs [] = {};
        for (unsigned int i = 0; i < numCameras; i++)
        {
            cout << i << endl;
            try
            {
                pCam = camList.GetByIndex(i);
                cout << "pCam->GetTLDeviceNodeMap()..." << endl;
                //NBL: From here....
                // Retrieve TL device nodemap and print device information
                INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
                cout << "pCam->Init()...." << endl;
                // Initialize camera
                pCam->Init();
                cout << "pCam->GetNodeMap()...." << endl;
                // Retrieve GenICam nodemap
                // Retrieve GenICam nodemap
                INodeMap & nodeMap = pCam->GetNodeMap();

                CStringPtr deviceModelName = nodeMap.GetNode("DeviceModelName");
                deviceModelNameS = deviceModelName->GetValue().c_str();
                 
                //NBL: .... to here should be run ONCE for each camera.  THIS is 
                //where serial number acquisition should happen.
                
            }
            catch (Spinnaker::Exception &e)
            {
                cout << "Error: " << e.what() << endl;
                return -1;
            }
            if(desiredSN == deviceModelNameS){
                break;
            }
            //pCam = camList.GetByIndex(i);
            //myCameras[i] = anyCamera(i);
            //deviceRegistration (camList.GetByIndex(i), i);
            
        }
        cout << "device Model Name = " << deviceModelNameS<<endl;
    }
    
    
      
    cout << "Finished camera registration!" << endl << endl; 
    while (ros::ok())
    {
        //NBL: Acquire and record (Save) camera data (PPM).
        run_camera(camList, numCameras);    
        ros::spinOnce();
        loop_rate.sleep();  
    } 
 
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    return 0;
}

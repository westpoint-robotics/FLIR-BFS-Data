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


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

//NBL: Typing issue?
std_msgs::String record;
CameraList camList;
//string deviceSNArry [] = {};
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
    string desiredSN = "Blackfly BFLY-PGE-50S5C";
    string deviceModelNameS = "";
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
    /*while (ros::ok())
    {
        //NBL: Acquire and record (Save) camera data (PPM).
        runCameras(numCameras);//camList, numCameras);    
        ros::spinOnce();
        loop_rate.sleep();  
    }
    CameraPtr pCam;*/
    /*for (unsigned int i = 0; i < numCameras; i++){
        pCam = camList.GetByIndex(i);
        // Deinitialize camera
        pCam->DeInit(); 
    }*/
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    return 0;
}

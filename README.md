# FLIR-BFS-Data
C++, and or Python code for pulling data from FLIR camera according to user spec.

Spinnaker SDK for FLIR cameras
Linux downloads can be found here: https://flir.app.boxcn.net/v/SpinnakerSDK/folder/69083919457
Choose the Ubuntu16.04, or Ubuntu18.04 folder depending on the OS you are using.  Within each folder is a tar file for an ARM or AMD computer; you will need to download the one that matches your machine's architecture.  Once you have downloaded and extracted the tar, you should find a README in the resulting file.  The README covers dependencies for 16.04 and 18.04, so be careful as you read through to follow the steps that are relevant to the OS you are using.

In addition to getting the Spinnaker SDK on your machine, you will need to make sure ROS (http://wiki.ros.org/ROS/Installation) in installed.  You will also have to create a catkin workspace, which will have a source (src) folder where your ROS packages live.  Your ROS packages will have their own src folders, and that is where your source code (C++, Python) will live.  The way this publication/subscription system works, the subscriber node (camera_subscriber.cpp) will be on a companion computer (e.g. Gigabyte Brix) for a UAV, and the publisher will be on a ground station machine.  Each machine will need to have the above setup in place.

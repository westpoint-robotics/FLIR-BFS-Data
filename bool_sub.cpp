#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

//NBL: Typing issue?
std_msgs::String record;

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
 
    ROS_INFO("I heard: [%s]", record.data.c_str());//msg->data.c_str());
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
  ros::Subscriber sub = n.subscribe("record", 1000, chatterCallback);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    
    if(record.data == "1"){
        //NBL: Acquire and record (Save) camera data (PPM).
        std::cout << "record is true - " << record << std::endl << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();  
  } 
  //std::cout << "Subscriber established." << std::endl;  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin();
  //ros::spinOnce();//Put this at the bottom of while() loop, and this does your callback update.... only in C++

  return 0;
}

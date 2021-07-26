// #include "ros/ros.h"
// #include "service_shelf/shelf_statu.h"

// #include <iostream>

// int main(int argc,char **argv)
// {
//   ros::init(argc,argv,"client_node");
//   ros::NodeHandle nh;

//   ros::ServiceClient client =
//   nh.serviceClient<service_shelf::shelf_statu>("add_two_ints");
//   service_shelf::shelf_statu srv;
  
//   while(ros::ok())
//   {
//     long int a_in,b_in;
//     std::cout<<"please input a and b:";
//     std::cin>>a_in>>b_in;

//     srv.request.a = a_in;
//     srv.request.b = b_in;
//     if(client.call(srv))
//     {
//       ROS_INFO("sum=%ld",(long int)srv.response.sum);
//     }
//     else
//     {
//       ROS_INFO("failed to call service add_two_ints");
//     }
//   }
//   return 0;
// }
#include <service_shelf/DoDishesAction.h> 
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<service_shelf::DoDishesAction> Client;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_client");
  Client client("do_dishes", true); // true -> don't need ros::spin()
  client.waitForServer(); // Waits for the ActionServer to connect to this client
  service_shelf::DoDishesGoal goal;
  // Fill in goal here
  goal.task=1;
  client.sendGoal(goal); // Sends a goal to the ActionServer
  client.waitForResult(ros::Duration(5.0)); // Blocks until this goal finishes
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean\n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
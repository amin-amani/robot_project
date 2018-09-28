#include "ros/ros.h"
#include <unistd.h>
#include <termios.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <std_srvs/Empty.h>
using namespace std;
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_teleop");
  ros::NodeHandle nh;

  ros::Publisher updownPub = nh.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 10);
  ros::Publisher fbPub = nh.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 10);
  ros::Publisher rlPub = nh.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 10);
  ros::ServiceClient vacuumOff = nh.serviceClient<std_srvs::Empty>("/vacuum_gripper/off");
  ros::ServiceClient vacuumOn = nh.serviceClient<std_srvs::Empty>("/vacuum_gripper/on");
  std_srvs::Empty emptyReq;
  std_msgs::Float64 updownMessage,rlMessage,fbMessage;
  updownMessage.data =0.00;
  rlMessage.data=0;
  fbMessage.data=0;
  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
     int c = getch();


ROS_INFO("msg{%c}",c);
if(c=='w')
{
fbMessage.data +=.001;
}
else if(c=='x')
{
fbMessage.data -=.001;
}
else if(c=='d')
{
rlMessage.data +=.001;
}
else if(c=='a')
{
rlMessage.data -=.001;
}
else if(c=='e')
{
updownMessage.data +=.001;
}
else if(c=='q')
{
updownMessage.data -=.001;
}
else if(c=='z')
{
vacuumOn.call(emptyReq);
}
else if(c=='c')
{

vacuumOff.call(emptyReq);
}


updownPub.publish(updownMessage);
rlPub.publish(rlMessage);
fbPub.publish(fbMessage);

    ros::spinOnce();

    //loop_rate.sleep();
  }

  return 0;
}

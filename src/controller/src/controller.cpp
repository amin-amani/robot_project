#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
ros::Publisher gripperPub ;
ros::Publisher rotPub ;
ros::Publisher updownPub ;
ros::Publisher fbPub;
ros::Publisher rlPub;
ros::ServiceClient vacuumOff ;
ros::ServiceClient vacuumOn;
  std_msgs::Float64 updownMessage,rlMessage,fbMessage,rotMessage,gripperMessage;


void chatterCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if( msg->poses.size()!=1)
    {return;}

double kp=0.00001;
    double yError= msg->poses[0].position.y-400.0;
    double xError= msg->poses[0].position.x-405.0;

if(yError<0)rlMessage.data+=fmin(fabs(kp*yError),0.002);
if(yError>0)rlMessage.data-=fmin(fabs(kp*yError),0.002);

if(xError>0)fbMessage.data+=fmin(fabs(kp*xError),0.002);
if(xError<0)fbMessage.data-=fmin(fabs(kp*xError),0.002);

ROS_INFO("%f object on x=%f y=%f", fmin(kp*yError,0.002),msg->poses[0].position.x,msg->poses[0].position.y);
rlPub.publish(rlMessage);
fbPub.publish(fbMessage);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
   gripperPub = nh.advertise<std_msgs::Float64>("/rrbot/joint5_position_controller/command", 10);
     rotPub = nh.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 10);
     updownPub = nh.advertise<std_msgs::Float64>("/rrbot/joint4_position_controller/command", 10);
     fbPub = nh.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 10);
     rlPub = nh.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 10);

    ros::Subscriber sub = nh.subscribe("/detected_objects", 1000, chatterCallback);

    ros::spin();

    return 0;
}

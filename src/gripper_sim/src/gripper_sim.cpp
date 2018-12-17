#include "ros/ros.h"
#include "std_msgs/String.h"
#include <QObject>
#include <QDebug>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include<gazebo_msgs/ApplyBodyWrench.h>

#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include "pidcontroller.h"

double ferq=100;
float command=0;
  QString nearestLink="";
ros::ServiceClient client;
// PIDController( double dt, double max, double min, double Kp, double Kd, double Ki );
PIDController pidController(1/ferq,5000,-3000,50000,10000,0.5);

geometry_msgs::Pose GetLinkPosition(QString linkName ,const gazebo_msgs::LinkStates::ConstPtr linkStates)
{
  geometry_msgs::Pose result;
const std::vector<std::string> &names = linkStates->name;
const std::vector<geometry_msgs::Pose> positions = linkStates->pose;


for(int i=0;i<names.size();i++)
{


  if(QString::fromStdString( names[i]).contains(linkName))
  {

    return  positions[i];

  }
}

return result;
}
 void GetNearestLink(  geometry_msgs::Pose referencePostion,QString modelName  ,const gazebo_msgs::LinkStates::ConstPtr linkStates,double &distance,QString &nearestLink )
{
double resulDistance=999999999;
QString resultLink;
const std::vector<std::string> &names = linkStates->name;
const std::vector<geometry_msgs::Pose> positions = linkStates->pose;

for(int i=0;i<names.size();i++)
{
 if(QString::fromStdString( names[i]).contains(modelName))continue;
 double diffz=(referencePostion.position.z-positions[i].position.z  );
  double diffxyz=pow((referencePostion.position.x-positions[i].position.x  ),2)+pow((referencePostion.position.y-positions[i].position.y)  ,2)+pow(diffz,2);
 // qDebug()<<QString::fromStdString( names[i])<<diffxyz;
 if(diffz>=0 && diffxyz<resulDistance){
   resulDistance=diffxyz;
   resultLink=QString::fromStdString( names[i]);

 }


}
nearestLink=resultLink;
distance=resulDistance;
qDebug()<<nearestLink<<distance;

}

 void ApplyWrench(QString bodyName,double force)
 {
   gazebo_msgs::ApplyBodyWrench bw;
qDebug()<<bodyName<<force;
 bw.request.body_name=bodyName.toStdString();
 bw.request.reference_frame=bodyName.toStdString();
 bw.request.duration.fromSec(.5);
 bw.request.start_time.fromSec(0);
 bw.request.wrench.force.z=force;
     client.call(bw);



 }
void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
//const std::vector<std::string> &names = msg->name;
//const std::vector<geometry_msgs::Pose> psitions = msg->pose;
  double distance=0;

  geometry_msgs::Pose parentPosition=GetLinkPosition("robot::niddle",msg);
  GetNearestLink(parentPosition,"robot",msg,distance,nearestLink);
 command=pidController.Calculate(parentPosition.position.z,distance);

//qDebug()<<QString::fromStdString( names[0])<<psitions[0].position.x<<psitions[0].position.y<<psitions[0].position.z;
ROS_INFO("I heard");

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_sim");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1, &chatterCallback);
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrenchRequest>("/gazebo/apply_body_wrench");


  ros::Rate loop_rate(ferq);

  while (ros::ok())
  {
    std_msgs::String msg;
    //msg.data = "hello world";
    ApplyWrench(nearestLink,command);
   // chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

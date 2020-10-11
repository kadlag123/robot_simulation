#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>
float kp=0,ki=0,kd=0;
float prev_error=0,control_signal,delta_error=0,curr_error=0,cum_error=0;
ros::Publisher pub;

void chatterCallback(const std_msgs::Int32::ConstPtr& msg){
  ROS_INFO("error is [%d] \n",msg->data);
  geometry_msgs::Twist vel;
  curr_error=msg->data;
  cum_error=curr_error+prev_error;
  delta_error=curr_error-prev_error;
  control_signal=(kp*curr_error+ki*cum_error+kd*delta_error);
  ROS_INFO("control input is [%f]",control_signal/100);
  ROS_INFO("kp ki kd are [%f] [%f] [%f]",kp,ki,kd);
  prev_error=curr_error;
  vel.linear.x=0.2;//0.075;
  vel.angular.z=-(control_signal/110);
  pub.publish(vel);

  ros::spinOnce();

}
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f \n", 
            config.kp, config.kd, config.ki);
  kp=config.kp;
  ki=config.ki;
  kd=config.kd;
  ROS_INFO("kp kd ki are {%f] [%f] [%f]",kp,kd,ki);
  ros::spinOnce();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");
   ros::NodeHandle n;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber sub = n.subscribe("/error", 10, chatterCallback);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

#include <string>
#include <ros/ros.h>
#include <encoder/encoder.h>
#include "serial/cmd.h"

int main(int argc, char **argv)
{
  //initial node
  ros::init(argc, argv, "encoder");

  ros::NodeHandle nh;

  //define encoder_msg
  encoder::encoder msg;
  //define srv
  serial::cmd srv;

  //create publisher
  ros::Publisher pub = nh.advertise<encoder::encoder>("encoder", 50);
  //subscribe service
  ros::ServiceClient client = nh.serviceClient<serial::cmd>("serial");

  //define the publish rate 
  ros::Rate loop_rate(30.0);

  //publish
  while (ros::ok())
  {
    srv.request.cmd = "e\r";
    if(client.call(srv)){
      for(int i = 0;i<srv.response.reply.length();++i){
        if(srv.response.reply[i] == '|'){
          msg.header.stamp = ros::Time::now();
          msg.left_ticks = atol(srv.response.reply.substr(0,i).c_str());
          msg.right_ticks = atol(srv.response.reply.substr(i+1).c_str());
          pub.publish(msg);
          break;
        }
      }
    }
    else
	  {
		  ROS_ERROR("Encoder: Failed to call serial!");
	  }

    //sleep to match the publish rate
    loop_rate.sleep();
  }

  return 0;
} 
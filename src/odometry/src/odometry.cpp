#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <cv_bridge/cv_bridge.h>
//message_filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//msgs
#include <encoder/encoder.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#define depth_max 10.0
#define depth_min 0.5

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace encoder;  //message defined by customer must claim the namespace when used?
using namespace message_filters;

//PoseStamped message: 
// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// geometry_msgs/Pose pose
//   geometry_msgs/Point position
//     float64 x
//     float64 y
//     float64 z
//   geometry_msgs/Quaternion orientation
//     float64 x
//     float64 y
//     float64 z
//     float64 w

//define messages//
encoder::encoder pre_encoder_msg;
Image pre_color_im, pre_depth_im;
PoseStamped pre_pose_msg, cur_pose_msg;
//odometry semaphore and flag
bool semaphore = true, first_time = true;

void callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const encoder::encoderConstPtr& encoder)
{
  // Solve all of perception here...
  pre_pose_msg.pose.position.x = 0;
}

int main(int argc, char **argv)
{
  //deifne node
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;

  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 50);
  //define the publish rate 
  ros::Rate loop_rate(30.0);

  //initial Pose message
  pre_pose_msg.header.stamp = ros::Time::now();
  pre_pose_msg.pose.position.x = 0;
  pre_pose_msg.pose.position.y = 0;
  pre_pose_msg.pose.position.z = 0;
  pre_pose_msg.pose.orientation.w = 1;
  pre_pose_msg.pose.orientation.x = 0;
  pre_pose_msg.pose.orientation.y = 0;
  pre_pose_msg.pose.orientation.z = 0;

  //define subscriber using message filter
  message_filters::Subscriber<Image> camera_color_sub(nh, "camera_color", 50);
  message_filters::Subscriber<Image> camera_depth_sub(nh, "camera_depth", 50);
  message_filters::Subscriber<encoder::encoder> encoder_sub(nh, "encoder", 100);

  typedef sync_policies::ExactTime<Image, Image, encoder::encoder> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), camera_color_sub, camera_depth_sub, encoder_sub);
  //TimeSynchronizer<Image, Image, encoder::encoder> sync(camera_color_sub, camera_depth_sub, encoder_sub, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::AsyncSpinner spinner(4);
  spinner.start();

  //publish the pose message
  while (ros::ok())
  {
    //...
    ROS_INFO("I am here.");

    //sleep to match the publish rate
    loop_rate.sleep();
  }

  return 0;
}
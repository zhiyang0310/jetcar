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
//using namespace encoder;  //message defined by customer must claim the namespace when used?
using namespace message_filters;

void callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const encoder::encoderConstPtr& encoder)
{
  // Solve all of perception here...
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;

  message_filters::Subscriber<Image> camera_color_sub(nh, "camera_color", 50);
  message_filters::Subscriber<Image> camera_depth_sub(nh, "camera_depth", 50);
  message_filters::Subscriber<encoder::encoder> encoder_sub(nh, "encoder", 100);

  typedef sync_policies::ExactTime<Image, Image, encoder::encoder> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), camera_color_sub, camera_depth_sub, encoder_sub);
  //TimeSynchronizer<Image, Image, encoder::encoder> sync(camera_color_sub, camera_depth_sub, encoder_sub, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::MultiThreadedSpinner(); 
  return 0;
}
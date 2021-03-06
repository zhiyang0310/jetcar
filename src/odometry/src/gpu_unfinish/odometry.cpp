//#include <stdlib>
#include <thread>
#include <mutex>
#include <condition_variable>
//////////////////////////////
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/cuda_stream_accessor.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"

#include <cv_bridge/cv_bridge.h>

#include <cblas.h>
#include "../include/matrix_compute.h"
//message_filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//msgs
#include "encoder/encoder.h"
#include <sensor_msgs/Image.h>
#include "odometry/pose2DStamped.h"

#include <vector>

#define depth_max         10000.0
#define depth_min         500.0
#define length_per_tick        10.0
#define d                 0.155
#define point_num         50

using namespace cv;
using namespace cuda;
using namespace sensor_msgs;
//using namespace nav_msgs;
using namespace encoder;  //message defined by customer must claim the namespace when used?
using namespace message_filters;

////////////////////////////////////////////////////////////////////////////////////////////////////
static cv::cuda::GpuMat pre_pose_gpu = cv::cuda::GpuMat(3,1,CV_32F,cv::Scalar(0)),
                        cur_pose_gpu = cv::cuda::GpuMat(3,1,CV_32F,cv::Scalar(0));
static long pre_left_ticks = 0, pre_right_ticks = 0, delta_left_ticks = 0, delta_right_ticks = 0;
static float s = 0, s_left = 0, s_right = 0, delta_x = 0, delta_y = 0, delta_theta = 0;
static cv_bridge::CvImageConstPtr color_image_bridge, depth_image_bridge;
static cv::cuda::GpuMat pre_image_gpu, cur_image_gpu;
static cv::cuda::GpuMat pre_depth_gpu, cur_depth_gpu;
static cv::cuda::GpuMat pre_mask_gpu, cur_mask_gpu;

//define ORB detector and matcher
static cv::Ptr<cv::cuda::ORB> ORB_operator = cv::cuda::ORB::create(500, 1.2f, 6, 31, 0, 2, 0, 31, 20,true);
static cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
static std::vector<cv::DMatch> matches, good_matches;
//define keypoints and descriptions
static cv::cuda::GpuMat pre_keypoints, cur_keypoints;
static cv::cuda::GpuMat pre_descriptions, cur_descriptions;
//kalman filter
static float mu[3] = {0.0, 0.0, 0.0}, sigma[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, 
             sigma_inverse[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},  
             G[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, H_transpose[3*2*point_num];
static const float R[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, 
                   Q[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                   K[9] = {},
                   K_inverse[9] = {};

//////////////////////////////////////////////////////////////////////////////////////////////////////

//define messages//
static ros::Time pre_time;
static odometry::pose2DStamped msg;
//define functions
void encode_thread(const encoder::encoderConstPtr& encoder);
void camera_thread(const ImageConstPtr& color_image, const ImageConstPtr& depth_image);
//flags
bool first = true, data_prepared = true;
//define mutex and condition_varible
std::mutex mut_pub;
std::condition_variable cond_pub;

void encode_thread(const encoder::encoderConstPtr& encoder){
  pre_left_ticks = encoder->left_ticks;
  pre_right_ticks = encoder->right_ticks;
  //s:
  s_left = delta_left_ticks*length_per_tick;
  s_right = delta_right_ticks*length_per_tick;
  s = 0.5*(s_left + s_right);
  //x,y:
  delta_x = s*cos(mu[2]);
  delta_y = s*sin(mu[2]);
  mu[0] += delta_x;
  mu[1] += delta_y;
  //compute G
  G[6] = -delta_y;
  G[7] = delta_x;
  //delta_theta:
  delta_theta = (s_right - s_left)/d;
  //update theta:
  mu[2] += delta_theta;
  //estimate covariance
  strmm(G, sigma, CblasLeft, CblasUpper, CblasNoTrans, CblasUnit);
  strmm(G, sigma, CblasRight, CblasUpper, CblasTrans, CblasUnit);
  sigma[0] += R[0];sigma[4] += R[4];sigma[8] += R[8];
  //compute sigma_inverse to implement Sherman/Morrison formula
  for(int i = 0; i < 9; ++i){
    sigma_inverse[i] = sigma[i];
  }
  if(matrix_inverse(sigma_inverse, 3)){
    ROS_INFO("Odometry: Compute sigma_inverse failed, exit.");
    exit(1);
  }
}

void camera_thread(const ImageConstPtr& color_image, const ImageConstPtr& depth_image){
  // upload images
    color_image_bridge = cv_bridge::toCvShare(color_image, "bgr8");
    depth_image_bridge = cv_bridge::toCvShare(depth_image, "mono16");
    cur_image_gpu.upload(color_image_bridge->image);
    cur_depth_gpu.upload(depth_image_bridge->image);
    ///////////////////////////////////////////////////////////////
    cv::cuda::threshold(cur_depth_gpu, cur_mask_gpu, depth_min, depth_max, CV_THRESH_BINARY);
    ORB_operator -> detectAndComputeAsync(cur_image_gpu, cur_mask_gpu, cur_keypoints, cur_descriptions);
    //match features
    matcher->match(pre_descriptions, cur_descriptions, matches);
    
    int sz = matches.size();
		double max_dist = 0; double min_dist = 100;
 
		for (int i = 0; i < sz; ++i)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}
 
		for (int i = 0; i < sz; ++i)
		{
			if (matches[i].distance < 2*min_dist)
			{
				good_matches.push_back(matches[i]);
			}
		}
    //
}

void compute_H_transpose(){
  
}

void callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const encoder::encoderConstPtr& encoder, const ros::Publisher& pub)
{
  //judge the header.stamp is reasonable
  if(encoder->header.stamp <= pre_time){
    return;
  }
  else{
    pre_time = encoder->header.stamp;
  }
  //first execute
  if(first){
    color_image_bridge = cv_bridge::toCvShare(color_image, "bgr8");
    depth_image_bridge = cv_bridge::toCvShare(depth_image, "mono16");
    pre_image_gpu.upload(color_image_bridge->image);
    pre_depth_gpu.upload(depth_image_bridge->image);
    ///////////////////////////////////////////////////////////////////
    cv::cuda::threshold(pre_depth_gpu, pre_mask_gpu, depth_min, depth_max, CV_THRESH_BINARY);
    ORB_operator -> detectAndComputeAsync(pre_image_gpu, pre_mask_gpu, pre_keypoints, pre_descriptions);
    ROS_INFO("Odometry: Initialize done.");
    first = false;
    return;
  }
  delta_left_ticks = encoder->left_ticks - pre_left_ticks;
  delta_right_ticks = encoder->right_ticks - pre_right_ticks;
  if(abs(delta_left_ticks)<5 && abs(delta_right_ticks)<5){return;}
  else{
    //run threads
    std::thread t1(camera_thread, color_image, depth_image);
    encode_thread(encoder);
    t1.join();










    msg.header.stamp = pre_time;
    msg.pose.x = mu[0];
    msg.pose.y = mu[1];
    msg.pose.theta = mu[2];
    pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  //deifne node
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;

  //define publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("pose", 50);
  //define the publish rate 
  ros::Rate loop_rate(30.0);

  //define subscriber using message filter
  message_filters::Subscriber<Image> camera_color_sub(nh, "camera_color", 10);
  message_filters::Subscriber<Image> camera_depth_sub(nh, "camera_depth", 10);
  message_filters::Subscriber<encoder::encoder> encoder_sub(nh, "encoder", 20);

  typedef sync_policies::ExactTime<Image, Image, encoder::encoder> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_color_sub, camera_depth_sub, encoder_sub);
  //TimeSynchronizer<Image, Image, encoder::encoder> sync(camera_color_sub, camera_depth_sub, encoder_sub, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, pub));

  ros::spin();

  //while(ros::ok()){
  //  std::unique_lock<std::mutex> lk(mut_pub);
  //  cond_pub.wait(lk, [&data_prepared]{if(data_prepared == true){data_prepared = false;return true}
  //                                     else{return data_prepared;} });
  //  pub.publish(msg);
  //  loop_rate.sleep();
  //}

  return 0;
}

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

// #include <iostream>
// #include <opencv2/highgui/highgui.hpp>  
// #include <opencv2/imgproc/imgproc.hpp>

#define depth_max 10.0
#define depth_min 0.5
#define image_height 480
#define image_width 640

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

int main(int argc, char **argv)
{
  //set camera parameters
  rs2::pipeline pipe;
  rs2::config pipe_config;
  pipe_config.enable_stream(RS2_STREAM_COLOR,image_width,image_height,RS2_FORMAT_BGR8,30);
  pipe_config.enable_stream(RS2_STREAM_DEPTH,image_width,image_height,RS2_FORMAT_Z16,30);
  rs2::pipeline_profile profile = pipe.start();

  //obtain depth_scale
  float depth_scale = get_depth_scale(profile.get_device());

  //set the stream which we plan to align to
  rs2_stream align_to = RS2_STREAM_COLOR;
  rs2::align align(align_to);

  //a few seconds to settle the whiteblance
  for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

  //define two cv::Mat to get rgb and depth image from camera
  cv::Mat aligned_depth_image, aligned_color_image;
  //convert cv::Mat into two CvImageptr
  cv_bridge::CvImagePtr color_image, depth_image;
  color_image = boost::make_shared< cv_bridge::CvImage >();
  depth_image = boost::make_shared< cv_bridge::CvImage >();
  //define info
  color_image->encoding = sensor_msgs::image_encodings::RGB8;
  depth_image->encoding = sensor_msgs::image_encodings::MONO16;

  //define node name
  ros::init(argc, argv, "camera");

  //initial node
  ros::NodeHandle nh;

  //define messages
  sensor_msgs::Image color_msg;
  sensor_msgs::Image depth_msg;

  //create publisher
  ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>("camera_color", 50);
  ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("camera_depth", 50);

  //define the publish rate
  ros::Rate loop_rate(30.0);

  //publish msg
  while (ros::ok())
  {
    rs2::frameset frameset = pipe.wait_for_frames();
    if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
    {
        //If the profile was changed, update the align object, and also get the new device's depth scale
        profile = pipe.get_active_profile();
        align_to = find_stream_to_align(profile.get_streams());
        align = rs2::align(align_to);
        depth_scale = get_depth_scale(profile.get_device());
    }

    //Get processed aligned frame
    auto processed = align.process(frameset);

    // Trying to get both other and aligned depth frames
    rs2::video_frame aligned_color_frame = processed.first(align_to);
    rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

    //If one of them is unavailable, continue iteration
    if (!aligned_depth_frame || !aligned_color_frame)
    {
        continue;
    }

    //stamp time
    color_image->header.stamp = ros::Time::now();
    depth_image->header.stamp = ros::Time::now();
    //hold the images
    aligned_depth_image = cv::Mat(cv::Size(image_width,image_height),CV_16UC1,(void*)aligned_depth_frame.get_data(),cv::Mat::AUTO_STEP);
    aligned_color_image = cv::Mat(cv::Size(image_width,image_height),CV_8UC3,(void*)aligned_color_frame.get_data(),cv::Mat::AUTO_STEP);
    color_image->image = aligned_color_image;
    depth_image->image = aligned_depth_image;
    
    ///////////////////////////// see the images ///////////////////////////////////
    // std::cout<<depth_image->toImageMsg()->header.stamp<<std::endl;
    // std::cout<<depth_image->toImageMsg()->encoding<<std::endl;
    // std::cout<<depth_image->toImageMsg()->height<<std::endl;
    // std::cout<<depth_scale<<std::endl;
    // imwrite("./1.jpg", aligned_color_image);
    // imwrite("./2.jpg", 255*aligned_depth_image/10000);
    // imwrite("./3.jpg", 255*cv::Mat(cv::Size(image_width,image_height),CV_16UC1,(void*)frameset.get_data(),cv::Mat::AUTO_STEP)/10000);
    // break;
    //////////////////////////////////////////////////////////////////////////////////

    //publish
    color_pub.publish(color_image->toImageMsg());
    depth_pub.publish(depth_image->toImageMsg());

    //Sleep to match the publish rate
    loop_rate.sleep();
  }
  return 0;
}

////////////////////////////define funtions////////////////////////////
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}
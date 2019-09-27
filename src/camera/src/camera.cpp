#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <camera/rgb.h>
#include <camera/depth.h>

#define depth_max 5.0
#define depth_min 0.5

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

int main(int argc, char **argv)
{
  //set camera parameters
  rs2::pipeline pipe;
  rs2::config pipe_config;
  pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
  pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
  rs2::pipeline_profile profile = pipe.start();

  //obtain depth_scale
  float depth_scale = get_depth_scale(profile.get_device());

  //set the stream which we plan to align to
  rs2_stream align_to = RS2_STREAM_COLOR;
  rs2::align align(align_to);

  //a few seconds to settle the whiteblance
  for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

  //define two pointer to get rgb and depth image from camera
  uint8_t *p;  //hold rgb
  uint16_t *q; //hold depth

  //define node name
  ros::init(argc, argv, "camera");

  //initial node
  ros::NodeHandle nh;

  //define msg
  camera::rgb rgb_msg;
  camera::depth depth_msg;

  //create publisher
  ros::Publisher rgb_pub = nh.advertise<camera::rgb>("camera_rgb", 50);
  ros::Publisher depth_pub = nh.advertise<camera::depth>("camera_depth", 50);

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

    //stamp image time
    rgb_msg.header.stamp = ros::Time::now();
    depth_msg.header.stamp = ros::Time::now();

    //prepare to publish
    rgb_msg.height = aligned_color_frame.as<rs2::video_frame>().get_height();
    rgb_msg.width = aligned_color_frame.as<rs2::video_frame>().get_width();
    p = (uint8_t*)aligned_color_frame.get_data(); 
    for(int i = 0;i<rgb_msg.height*rgb_msg.width;++i){
        rgb_msg.data.push_back(*(p + i));
    }

    depth_msg.height = aligned_depth_frame.as<rs2::video_frame>().get_height();
    depth_msg.width = aligned_depth_frame.as<rs2::video_frame>().get_width();
    q = (uint16_t*)aligned_depth_frame.get_data(); 
    for(int i = 0;i<depth_msg.height*depth_msg.width;++i){
        depth_msg.data.push_back(*(q + i));
    }
    
    rgb_pub.publish(rgb_msg);
    depth_pub.publish(depth_msg);

    //clear previous data
    rgb_msg.data.clear();
    depth_msg.data.clear();

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
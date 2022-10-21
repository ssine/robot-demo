#ifndef RB5_CAMERA_OCV
#define RB5_CAMERA_OCV
#include <stdio.h>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <glib.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <string>

class RbCamera : public rclcpp::Node
{
public:
  RbCamera();
  ~RbCamera();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_pub;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cam_compress_pub;

  // data
  typedef struct _CustomData
  {
    GstElement * pipeline;
    GstElement * source;
    GstElement * convert;
    GstElement * capsfiltersrc;
    GstElement * capsfilterapp;
    GstElement * appsink;
  } CustomData;

  void init();

  GstMessage * msg;
  GstBus * bus;
  GstStateChangeReturn ret;
  guint bus_id;

  // caps parameters for the camera
  int camera_id;
  int frame_rate;
  int width;
  int height;
  bool image_compress;

  std::string input_format;
  std::string output_format;


  CustomData data;
};

#endif

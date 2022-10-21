#include "rb_camera_ocv.h"
#include <sys/time.h>

/* Callback for appsink to parse the video stream and publish images. */
static GstFlowReturn processData(GstElement * sink, RbCamera * node)
{

  GstSample * sample;
  GstBuffer * buffer;
  GstMapInfo map_info;

  // Retrieve buffer
  g_signal_emit_by_name(sink, "pull-sample", &sample);

  if (sample) {
    buffer = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    GstStructure * caps_structure = gst_caps_get_structure(caps, 0);

    gboolean res;
    int width, height;
    const gchar * format;
    res = gst_structure_get_int(caps_structure, "width", &width);
    res |= gst_structure_get_int(caps_structure, "height", &height);
    format = gst_structure_get_string(caps_structure, "format");
    if (!res) {
      g_print("no dimensions");
    }

    // g_print("%s\n", gst_structure_to_string(caps_structure));

    if (!gst_buffer_map((buffer), &map_info, GST_MAP_READ)) {
      gst_buffer_unmap((buffer), &map_info);
      gst_sample_unref(sample);

      return GST_FLOW_ERROR;
    }

    timeval current_time;
    gettimeofday(&current_time, 0);
    std::cout << current_time.tv_sec << "." << current_time.tv_usec << std::endl;

    // Parse data from buffer, depending on the format, conversion might be needed.
    // cv::Mat frame_rgb = cv::Mat::zeros(width, height, CV_8UC3);
    cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (char *)map_info.data, cv::Mat::AUTO_STEP);
    // cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2);

    // Prepare ROS message.
    cv_bridge::CvImage bridge;

    // Publish camera image
    sensor_msgs::msg::Image cam_msg;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock().now();
    bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_rgb);
    bridge.toImageMsg(cam_msg);
    node->cam_pub->publish(cam_msg);

    // Publish camera frame
    if (node->image_compress) {
      sensor_msgs::msg::CompressedImage cam_compress_msg;
      cam_compress_msg.header.stamp = header.stamp;
      std::vector<int> p;
      p.push_back(cv::IMWRITE_JPEG_QUALITY);
      p.push_back(90);

      cv::Mat frame_bgr = cv::Mat::zeros(width, height, CV_8UC3);
      cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
      cv::imencode(".jpg", frame_bgr, cam_compress_msg.data, p);

      node->cam_compress_pub->publish(cam_compress_msg);
    }

    // free resources
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }


  return GST_FLOW_ERROR;
}

RbCamera::RbCamera()
: rclcpp::Node::Node("camera")
{
  this->declare_parameter("camera_id", 0);
  camera_id = this->get_parameter("camera_id").as_int();
  std::cout << "camera_id: " << camera_id << std::endl;

  this->declare_parameter("frame_rate", 30);
  frame_rate = this->get_parameter("frame_rate").as_int();
  std::cout << "frame_rate: " << frame_rate << std::endl;

  this->declare_parameter("width", 1920);
  width = this->get_parameter("width").as_int();
  std::cout << "width: " << width << std::endl;

  this->declare_parameter("height", 1080);
  height = this->get_parameter("height").as_int();
  std::cout << "height: " << height << std::endl;

  this->declare_parameter("input_format", "NV12");
  input_format = this->get_parameter("input_format").as_string();
  std::cout << "input_format: " << input_format << std::endl;

  this->declare_parameter("output_format", "RGB");
  output_format = this->get_parameter("output_format").as_string();
  std::cout << "output_format: " << output_format << std::endl;

  this->declare_parameter("topic_name", "camera" + std::to_string(camera_id));
  auto topic_name = this->get_parameter("topic_name").as_string();
  std::cout << "topic_name: " << topic_name << std::endl;

  this->declare_parameter("image_compress", true);
  image_compress = this->get_parameter("image_compress").as_bool();
  std::cout << "image_compress: " << image_compress << std::endl;

  cam_pub = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
  cam_compress_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    "camera/compressed",
    10);

  gst_init(0, nullptr);

  std::string input_caps = "video/x-raw,format=" + input_format +
    ",framerate=" + std::to_string(frame_rate) + "/1" +
    ",width=" + std::to_string(width) +
    ",height=" + std::to_string(height);

  std::string output_caps = "video/x-raw,format=" + output_format;

  std::cout << "input caps: " << input_caps << std::endl;
  std::cout << "output caps: " << output_caps << std::endl;

  if (camera_id == 0 or camera_id == 1) {
    data.source = gst_element_factory_make("qtiqmmfsrc", "source");
  } else {
    data.source = gst_element_factory_make("autovideosrc", "source");
  }

  data.capsfiltersrc = gst_element_factory_make("capsfilter", "capsfiltersrc");
  data.convert = gst_element_factory_make("videoconvert", "convert");
  data.capsfilterapp = gst_element_factory_make("capsfilter", "capsfilterapp");
  data.appsink = gst_element_factory_make("appsink", "sink");
  data.pipeline = gst_pipeline_new("rb5-camera");

  if (!data.pipeline || !data.convert || !data.source || !data.appsink ||
    !data.capsfiltersrc || !data.capsfilterapp)
  {
    g_printerr("Not all elements could be created.\n");
    return;
  }

  // Build pipeline
  gst_bin_add_many(
    GST_BIN(
      data.pipeline), data.source, data.convert, data.appsink, data.capsfiltersrc, data.capsfilterapp,
    nullptr);
  if (gst_element_link_many(
      data.source, data.capsfiltersrc, data.convert, data.capsfilterapp,
      data.appsink, nullptr) != TRUE)
  {
    g_printerr("Elements could not be linked.\n");
    gst_object_unref(data.pipeline);
    return;
  }

  if (camera_id == 0 or camera_id == 1) {
    g_object_set(G_OBJECT(data.source), "camera", camera_id, nullptr);
  }

  g_object_set(
    G_OBJECT(data.capsfiltersrc), "caps",
    gst_caps_from_string(input_caps.c_str()), nullptr);
  g_object_set(
    G_OBJECT(data.capsfilterapp), "caps",
    gst_caps_from_string(output_caps.c_str()), nullptr);
}

RbCamera::~RbCamera()
{
  gst_object_unref(bus);
  gst_element_set_state(data.pipeline, GST_STATE_NULL);
  gst_object_unref(data.pipeline);
}

void RbCamera::init()
{
  g_object_set(data.appsink, "emit-signals", TRUE, nullptr);
  g_object_set(G_OBJECT(data.source), "camera", camera_id, NULL);
  g_signal_connect(data.appsink, "new-sample", G_CALLBACK(processData), this);

  // play
  ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the pipeline to the playing state.\n");
    gst_object_unref(data.pipeline);
    return;
  }
  bus = gst_element_get_bus(data.pipeline);

  /* Wait until error or EOS */
  bus = gst_element_get_bus(data.pipeline);
  msg =
    gst_bus_timed_pop_filtered(
    bus, GST_CLOCK_TIME_NONE,
    (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  /* Parse message */
  if (msg != NULL) {
    GError * err;
    gchar * debug_info;

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error(msg, &err, &debug_info);
        g_printerr(
          "Error received from element %s: %s\n",
          GST_OBJECT_NAME(msg->src), err->message);
        g_printerr(
          "Debugging information: %s\n",
          debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
        break;
      case GST_MESSAGE_EOS:
        g_print("End-Of-Stream reached.\n");
        break;
      default:
        /* We should not reach here because we only asked for ERRORs and EOS */
        g_printerr("Unexpected message received.\n");
        break;
    }
    gst_message_unref(msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbCamera>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
}

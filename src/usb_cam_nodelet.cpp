#include <cstdlib>

#include <mutex>
#include <sstream>
#include <thread>

#include <camera_info_manager/camera_info_manager.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <system_error>

#include <usb_cam/usb_cam.hpp>

namespace usb_cam {

static const std::string DEFAULT_CAMERA_NAME = "head_camera";

static const std::string DEFAULT_VIDEO_DEVICE = "/dev/video0";
static const std::string DEFAULT_IO_METHOD = "mmap";
static const std::string DEFAULT_PIXEL_FORMAT = "yuyv";
static const std::string DEFAULT_COLOR_FORMAT = "yuv422p";

static const int DEFAULT_IMAGE_HEIGHT = 480;
static const int DEFAULT_IMAGE_WIDTH = 640;

static const int DEFAULT_FRAMERATE = 30;

class UsbCamNodelet : public nodelet::Nodelet {
public:
  UsbCamNodelet();
  ~UsbCamNodelet();

  void onInit() override;

private:
  bool callback_toggle_cap(std_srvs::SetBool::Request &, std_srvs::SetBool::Response &);
  bool callback_reset_ctrls(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &);
  void reset_ctrls();
  void spin();
  bool take_and_send_image();
  void get_params(ros::NodeHandle &);
  void register_ctrls(ros::NodeHandle &);

  sensor_msgs::ImagePtr img_;
  image_transport::CameraPublisher image_pub_;

  std::string video_device_name_;
  std::string camera_name_;
  std::string camera_info_url_;

  int image_width_;
  int image_height_;
  int framerate_;
  std::string io_method_name_;
  std::string pixel_format_name_;
  std::string color_format_name_;

  ddynamic_reconfigure::DDynamicReconfigure *ddr_;

  camera_info_manager::CameraInfoManager *cinfo_;

  UsbCam cam_;
  std::vector<ctrl> ctrls_;

  ros::ServiceServer service_toggle_capture_;
  ros::ServiceServer service_reset_ctrls_;

  std::thread publisher_thread_;
};

UsbCamNodelet::UsbCamNodelet() : img_(new sensor_msgs::Image()) {}

void UsbCamNodelet::onInit() {
  ros::NodeHandle pnh = getPrivateNodeHandle();

  image_transport::ImageTransport it(pnh);
  image_pub_ = it.advertiseCamera("image_raw", 1);

  get_params(pnh);

  cinfo_ = new camera_info_manager::CameraInfoManager(pnh, camera_name_, camera_info_url_);

  if (!cinfo_->isCalibrated()) {
    sensor_msgs::CameraInfo calibration;
    calibration.header.frame_id = img_->header.frame_id;
    calibration.width = image_width_;
    calibration.height = image_height_;
    cinfo_->setCameraInfo(calibration);
  }

  UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
  if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
    NODELET_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
    return;
  }

  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
    NODELET_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
    return;
  }

  UsbCam::color_format color_format = UsbCam::color_format_from_string(color_format_name_);
  if (color_format == UsbCam::COLOR_FORMAT_UNKNOWN) {
    NODELET_ERROR("Unknown color format '%s'", color_format_name_.c_str());
    return;
  }

  NODELET_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(),
               video_device_name_.c_str(), image_width_, image_height_, io_method_name_.c_str(),
               pixel_format_name_.c_str(), framerate_);

  cam_.start(video_device_name_.c_str(), io_method, pixel_format, color_format, image_width_,
             image_height_, framerate_);
  register_ctrls(pnh);

  service_toggle_capture_ =
      pnh.advertiseService("toggle_capture", &UsbCamNodelet::callback_toggle_cap, this);
  service_reset_ctrls_ =
      pnh.advertiseService("reset_ctrls", &UsbCamNodelet::callback_reset_ctrls, this);

  publisher_thread_ = std::thread(&UsbCamNodelet::spin, this);
}

UsbCamNodelet::~UsbCamNodelet() {
  service_toggle_capture_.shutdown();
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }
}

bool UsbCamNodelet::callback_toggle_cap(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res) {
  if (req.data) {
    cam_.start_capturing();
    res.success = cam_.is_capturing();
  } else {
    cam_.stop_capturing();
    res.success = ~cam_.is_capturing();
  }
  return true;
}

bool UsbCamNodelet::callback_reset_ctrls(std_srvs::Trigger::Request &,
                                         std_srvs::Trigger::Response &res) {
  reset_ctrls();
  res.success = true;
  return true;
}

void UsbCamNodelet::reset_ctrls() {
  NODELET_INFO("Resetting v4l ctrls to default values");
  for (auto &c : ctrls_) {
    c.value = c.default_value;
    cam_.set_ctrl(c.id, c.value);
  }
  ddr_->updatePublishedInformation();
}

void UsbCamNodelet::spin() {
  ros::Rate loop_rate(framerate_);
  while (ros::ok()) {
    if (cam_.is_capturing()) {
      if (!take_and_send_image()) {
        NODELET_WARN("USB camera did not respond in time.");
      }
    }
    loop_rate.sleep();
  }
}

bool UsbCamNodelet::take_and_send_image() {
  if (!cam_.get_image(img_->header.stamp, img_->encoding, img_->height, img_->width, img_->step,
                      img_->data)) {
    NODELET_ERROR("grab failed");
    return false;
  }

  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci->header.frame_id = img_->header.frame_id;
  ci->header.stamp = img_->header.stamp;

  image_pub_.publish(img_, ci);

  return true;
}

void UsbCamNodelet::get_params(ros::NodeHandle &nh) {
  nh.param("video_device", video_device_name_, DEFAULT_VIDEO_DEVICE);

  nh.param("image_width", image_width_, DEFAULT_IMAGE_WIDTH);
  nh.param("image_height", image_height_, DEFAULT_IMAGE_HEIGHT);
  nh.param("framerate", framerate_, DEFAULT_FRAMERATE);

  nh.param("io_method", io_method_name_, DEFAULT_IO_METHOD);
  nh.param("pixel_format", pixel_format_name_, DEFAULT_PIXEL_FORMAT);
  nh.param("color_format", color_format_name_, DEFAULT_COLOR_FORMAT);

  // load the camera info
  nh.param("camera_frame_id", img_->header.frame_id, DEFAULT_CAMERA_NAME);
  nh.param("camera_name", camera_name_, DEFAULT_CAMERA_NAME);
  nh.param("camera_info_url", camera_info_url_, std::string());
}

void UsbCamNodelet::register_ctrls(ros::NodeHandle &nh) {
  ddr_ = new ddynamic_reconfigure::DDynamicReconfigure(nh);

  cam_.query_ctrls(ctrls_);

  for (auto &c : ctrls_) {
    auto callback =
        boost::function<void(int)>([&cam = cam_, id = c.id](int val) { cam.set_ctrl(id, val); });

    if (c.menu.empty()) {
      ddr_->registerVariable(c.name, &c.value, callback, "", c.min_value, c.max_value);
    } else {
      ddr_->registerEnumVariable(c.name, &c.value, callback, "", c.menu, "");
    }

    nh.param(std::string("ctrls/") + c.name, c.value, c.value);
  }

  bool reset = nh.param("reset_ctrls", false);
  if (reset)
    reset_ctrls();

  for (auto &c : ctrls_) {
    std::string param_name = "ctrls/" + c.name;
    if (nh.hasParam(param_name)) {
      nh.getParam(param_name, c.value);
      cam_.set_ctrl(c.id, c.value);
    }
  }

  ddr_->publishServicesTopics();
}

} // namespace usb_cam

PLUGINLIB_EXPORT_CLASS(usb_cam::UsbCamNodelet, nodelet::Nodelet);
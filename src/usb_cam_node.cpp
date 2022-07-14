#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "usb_cam");

  // Start the usb_cam Nodelet
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(), "usb_cam/UsbCam", remap, nargv);

  ros::spin();

  return 0;
}
#include <usb.h>
#include <string>

#include "ros/ros.h"
#include "polhemus_ros_driver/calibrate.h"
#include "liberty.h"

using namespace std;

/* Vendor 0x0f44 -> Polhemus */
#define VENDOR 0xf44

/* Product 0xff20 -> Liberty v2 Motion Tracker (with USB 2.0 using an EzUSB fx2
   chip)  after loading the firmware (it has ProductId 0xff21 before) */
#define PRODUCT 0xff20

static struct usb_device *find_device_by_id(uint16_t vendor, uint16_t product) {
  struct usb_bus *bus;

  usb_find_busses();
  usb_find_devices();

  for (bus = usb_get_busses(); bus; bus = bus->next) {
    struct usb_device *dev;
    for (dev = bus->devices; dev; dev = dev->next) {
      if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
	return dev;
    }
  }
  return NULL;
}

static struct usb_dev_handle *get_polhemus_handle() {
  struct usb_device *dev;
  usb_dev_handle *handle;

  // Setup polhemus
  usb_init();

  dev = find_device_by_id(VENDOR, PRODUCT);
  if (!dev) {
    fprintf(stderr, "Could not find the Polhemus Liberty device.\n");
    abort();
  }

  handle = usb_open(dev);
  if (!handle) {
    fprintf(stderr, "Could not get a handle to the Polhemus Liberty device.\n");
    abort();
  }

  return handle;
}

bool calibrate(polhemus_ros_driver::calibrate::Request  &req, polhemus_ros_driver::calibrate::Response &res)
{
  usb_dev_handle *handle = get_polhemus_handle();
  string cmd = "";
  cmd = "b" + req.station + "," + req.azref + "," + req.elref + "," + req.rlref +  "," ;
  cmd += req.reset_origin ? "1" : "0";
  cmd += "\r";
  ROS_INFO("Calibration request: %s\n",cmd.c_str());
  liberty_send(handle, (char *)cmd.c_str());
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calibration", calibrate);
  ROS_INFO("Ready to calibrate the sensors.");
  ros::spin();

  return 0;
}
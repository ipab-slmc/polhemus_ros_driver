/*

  Communication library for a Polhemus Liberty v2 (tm) Motion tracker
  Copyright (C) 2008 Jonathan Kleinehellefort and Alexis Maldonado
  Intelligent Autonomous Systems Lab,
  Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen
  <kleinehe@cs.tum.edu>  <maldonad@cs.tum.edu>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

*/

/*
  This file is a modified version of polhemus_ros_node.cpp by Kleinehellefort and Maldonado.
  Modified by C. E. Mower, 2017.
*/

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <polhemus_ros_driver/liberty.hpp>
#include <polhemus_ros_driver/polhemus.hpp>
#include <polhemus_ros_driver/viper.hpp>
#include <polhemus_ros_driver/viper_protocol.h>
#include "polhemus_ros_driver/liberty_protocol.h"


/* Vendor 0x0f44 -> Polhemus */
#define VENDOR 0xf44

#define VPUSB_MAX_DISCOVERABLE 16

/* Product 0xff20 -> Liberty v2 Motion Tracker (with USB 2.0 using an EzUSB fx2
   chip)  after loading the firmware (it has ProductId 0xff21 before) */
#define LIBERTY_PRODUCT 0xff20
#define VIPER_PRODUCT 0xbf01


typedef struct _vp_usbdevinfo {
  int usbDevIndex;
  uint8_t usbBusNum;
  uint8_t usbDevNum;
  uint16_t usbVID;
  uint16_t usbPID;
  uint8_t usbNumInterfaces;
  uint8_t ep_in;
  uint8_t ep_out;
  uint16_t epout_maxPktsize;

  _vp_usbdevinfo() : usbDevIndex(-1), usbBusNum(0), usbDevNum(0), usbVID(0), usbPID(0), usbNumInterfaces(0), ep_in(0), ep_out(0), epout_maxPktsize(64) {};

}vp_usbdevinfo;

/* main loop running? */
static int go_on;

static void signal_handler(int s) {
  switch (s) {
  case SIGINT:
    go_on = 0;
    break;
  }
}

static void print_hex(FILE *stream, const char *buf, size_t size) {
  const char *c;
  for (c = buf; c != buf + size; ++c)
    fprintf(stream, "%02x:%c ", (unsigned char) *c, isprint(*c) ? *c : '.');
  fprintf(stream, "\n");
}

static void print_ascii(FILE *stream, const char *buf, size_t size) {
  const char *c;
  for (c = buf; c != buf + size; ++c)
    if (isprint(*c)) {
      fprintf(stream, "%c", *c);
    }
  fprintf(stream, "\n");
}

void ReleaseUSB(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinfo)
{
  if ((usbhnd == 0) || (*usbhnd == 0))
    return;

  int r = 0;
  for (int i = 0; i < usbinfo.usbNumInterfaces; i++)
    r = libusb_release_interface(*usbhnd, i);

  libusb_close (*usbhnd);
  *usbhnd = 0;
  usbinfo = vp_usbdevinfo();

}

void find_endpoints(libusb_config_descriptor *conf_desc, int iface, uint8_t & ep_in, uint8_t & ep_out,
                    uint16_t & out_pktsize)
{
  for (int j = 0; j < conf_desc->interface[iface].num_altsetting; j++)
  {
    for (int k = 0; k < conf_desc->interface[iface].altsetting[j].bNumEndpoints; k++)
    {
      const struct libusb_endpoint_descriptor *p_ep;
      p_ep = &conf_desc->interface[iface].altsetting[j].endpoint[k];

      if ((p_ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
          & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT))
      {
        if (p_ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
        {
          if (!ep_in)
            ep_in = p_ep->bEndpointAddress;
        } else
        {
          if (!ep_out)
          {
            ep_out = p_ep->bEndpointAddress;
            out_pktsize = p_ep->wMaxPacketSize;
          }
        }
      }

    }
  }
}


int create_vip_list(libusb_context* pctx, libusb_device **&devlist, uint16_t vid, uint16_t pid,
                     vp_usbdevinfo arrDevInfo[], std::size_t &arrcount)
{
  int r = 0;
  ssize_t devcount = libusb_get_device_list(pctx, &devlist);
  if (devcount < 0)
    return (int) devcount; // returns error code < 0
  //struct libusb_device *found = NULL;
  //struct libusb_device_handle *found_dev_handle = NULL;
  libusb_device *dev;
  int iFoundCount = 0;

  int iFoundArrIndex = -1;
  int i = 0;
  while ((dev = devlist[i]) != NULL)
  {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      break;

    if (desc.idVendor == vid && desc.idProduct == pid)
    {
      iFoundCount++;
      iFoundArrIndex++;
      // populate the array if size permits
      if (iFoundCount <= (int) arrcount)
      {
        vp_usbdevinfo * p = &arrDevInfo[iFoundArrIndex];
        p->usbDevIndex = i;
        p->usbBusNum = libusb_get_bus_number(dev);
        p->usbDevNum = libusb_get_device_address(dev);
        p->usbVID = vid;
        p->usbPID = pid;
      }
    }
    i++;
  }

  arrcount = iFoundCount;

  return r;
}

int discover_vip_pid(libusb_device_handle **usbhnd, vp_usbdevinfo &usbinfo, uint16_t vid, uint16_t pid)
{

  int r = 0;

  if (libusb_init (NULL))
    return -1;

  if (*usbhnd)
    ReleaseUSB(usbhnd, usbinfo);

  vp_usbdevinfo arrDevInfo[VPUSB_MAX_DISCOVERABLE];
  std::size_t arrcount = VPUSB_MAX_DISCOVERABLE;
  libusb_device **devlist;

  if ((r = create_vip_list(NULL, devlist, vid, pid, arrDevInfo, arrcount)) < 0)
    return r;

  for (int d = 0; d < (int) arrcount; d++)
  {
    libusb_device * dev = devlist[arrDevInfo[d].usbDevIndex];
    libusb_device_handle * handle = 0;
    libusb_config_descriptor *conf_desc = 0;
    uint8_t nb_ifaces = 0, claimed_ifaces = 0;
    uint8_t ep_in = 0, ep_out = 0;
    uint16_t out_pktsize = 0;

    if ((r = libusb_open(dev, &handle)))
    {
    } else if ((r = libusb_get_config_descriptor(dev, 0, &conf_desc)))
    {
      libusb_close(handle);
    } else
    {
      nb_ifaces = conf_desc->bNumInterfaces;
      claimed_ifaces = 0;
      for (uint8_t i = 0; (i < nb_ifaces) && (r == 0); i++)
      {
        if ((r = libusb_claim_interface(handle, (int) i)))
        {
        } else
        {
          claimed_ifaces++;

          find_endpoints(conf_desc, (int) i, ep_in, ep_out, out_pktsize);
        }
      }

      if (claimed_ifaces == nb_ifaces)
      {
        *usbhnd = handle;
        usbinfo = arrDevInfo[d];
        usbinfo.usbNumInterfaces = nb_ifaces;
        usbinfo.ep_in = ep_in;
        usbinfo.ep_out = ep_out;
        usbinfo.epout_maxPktsize = out_pktsize;

        break;
      }
    }

    // clean up failed attempt before trying the next one.
    if (claimed_ifaces != nb_ifaces)
    {
      for (int k = 0; k < nb_ifaces; k++)
        libusb_release_interface(handle, k);
    }
    if (handle)
      libusb_close(handle);

    libusb_free_config_descriptor(conf_desc);
  }

  libusb_free_device_list(devlist, 1);

  if (!usbhnd)
    r = -99;

  return r;
}


int main(int argc, char** argv) {
  libusb_device_handle *g_usbhnd = 0;
  vp_usbdevinfo g_usbinfo;
  int i, nstations;
  double x_hs, y_hs, z_hs;
  struct timeval tv;
  uint16_t product_id;
  std::string product_type;
  Polhemus *device;
  int retval = 0;

  // Setup ros
  ros::init(argc, argv, "polhemus_tf_broadcaster");
  ros::NodeHandle nh("/polhemus_tf_broadcaster");

  retval = discover_vip_pid(&g_usbhnd, g_usbinfo, VENDOR, VIPER_PRODUCT);

  if (retval)
  {
    //error connecting
    fprintf(stderr, "Error connecting to device.\n\n");
    return 1;
  }

  nh.getParam("/product_type", product_type);
  if (product_type == "liberty")
  {
	  product_id = LIBERTY_PRODUCT;
	  device = new Liberty();
	  fprintf(stderr, "Initialising liberty device.\n\n");
  }
  else if (product_type == "viper")
  {
	  product_id = VIPER_PRODUCT;
	  device = new Viper();
    fprintf(stderr, "Initialising Viper device.\n\n");
  }
  else
  {
	  fprintf(stderr, "Could not find a valid Polhemus device type on parameter server.\n");
	  abort();
  }

  device->endpoint_in = g_usbinfo.ep_in;
  device->endpoint_out = g_usbinfo.ep_out;

  device->device_handle = g_usbhnd;

  retval = device->device_reset();
  if (retval)
  {
    fprintf(stderr, "Error resetting device.\n\n");
    return 1;
  }

  device->device_binary_mode(); // activate binary mode
  retval = device->request_num_of_stations();
  if (retval)
  {
    fprintf(stderr, "Error reading number of stations.\n\n");
    return 1;
  }
  else
  {
    fprintf(stderr, "Found %d stations.\n\n", device->station_count);
    nstations = device->station_count;
  }

  /* define which information to get per sensor (called a station
     by polhemus)

     o* applies to all stations

     if this is changed, the station_t struct or (***) below has to be edited accordingly 
  */
  retval = device->define_quat_data_type();
  printf("Setting data type to quaternion\n");
  if (retval)
  {
    fprintf(stderr, "Error setting data type.\n\n");
    return 1;
  }

  retval = device->set_source(0);
  printf("Setting source to 0\n");
  if (retval)
  {
    fprintf(stderr, "Error source to 0.\n\n");
    // return 1;
  }

  // Calibration service
  ros::ServiceServer service = nh.advertiseService("calibration", &Polhemus::calibrate_srv, device);
  printf("Service ready to calibrate the sensors.\n");

  ros::ServiceServer service2 = nh.advertiseService("persistent_cfg", &Polhemus::persist_srv, device);
  printf("Service ready to persist cfg.\n");

  /* set output hemisphere -- this will produce a response which we're
     ignoring
  */
  nh.getParam("/x_hs", x_hs);
  nh.getParam("/y_hs", y_hs);
  nh.getParam("/z_hs", z_hs);

  printf("Setting the output hemisphere\n");
  retval = device->set_hemisphere(x_hs, y_hs, z_hs);
  if (retval)
  {
    fprintf(stderr, "Error setting hemisphere.\n\n");
    return 1;
  }

  /* switch output to centimeters */
  //liberty_send(handle, "u1\r");
//  device->device_clear_input(); //right now, we just ignore the answer

  device->generate_data_structure();

  /* set up signal handler to catch the interrupt signal */
  signal(SIGINT, signal_handler);

  go_on = 1;

  /* enable continuous mode (get data points continously) */
  printf("Enabling continuous data mode...\n");
  retval = device->device_data_mode(DATA_CONTINUOUS);
  if (retval)
  {
    fprintf(stderr, "Error setting data mode to continuous.\n\n");
    return 1;
  }

  gettimeofday(&tv, NULL);
  printf("Begin time: %d.%06d\n", (unsigned int) (tv.tv_sec), (unsigned int) (tv.tv_usec));

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Rate rate(240);

  // Start main loop
  while(ros::ok()) {
    if (go_on == 0)
      break;

    // Update polhemus
    // (***)
    int sensor_count = device->receive_pno_data_frame();
    if (sensor_count > 0)
    {
      /* Note: timestamp is the time in ms after the first read to the
         system after turning it on
         at 240Hz, the time between data sample is 1/240 = .00416_
         seconds.
         The framecount is a more exact way of finding out the time:
         timestamp = framecount*1000/240 (rounded down to an int)*
      */

      // Header info - acquired at same time = same timestamp
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "polhemus_base";

      for (i=0; i < sensor_count; i++)
      {
        //transformStamped.child_frame_id = "polhemus_station_" + std::to_string(i+1);
        device->fill_pno_data(&transformStamped, i);

        // Broadcast frame
        if (!retval)
        {
          //transformStamped.transform.translation.y = -transformStamped.transform.translation.y;
          br.sendTransform(transformStamped);
        }
      }
    }
    
    ros::spinOnce();
    rate.sleep();
  }

  printf("OUT OF THE LOOP! ros NOT OK!");
  // Shutdown
  device->device_data_mode(DATA_RESET); // stop continuous mode
  libusb_close(g_usbhnd);
  delete device;

  return 0;
}

/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/


#include <string.h>
#include <libusb-1.0/libusb.h>
#include "polhemus_ros_driver/polhemus.hpp"
#include <ros/console.h>


#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif


Polhemus::Polhemus(std::string name, uint16_t rx_buffer_size, uint16_t tx_buffer_size):
  name(name), g_rxbuf(new uint8_t[rx_buffer_size]), g_txbuf(new uint8_t[tx_buffer_size])
{
}

Polhemus::~Polhemus(void)
{
}

int Polhemus::count_bits(uint16_t v) {
  int c;
  for (c = 0; v; c++)
    {
      v &= v - 1; // clear the least significant bit set
    }
  return c;
}

int Polhemus::device_write(uint8_t *buf, int size, int timeout)
{
  int nActual = 0;
  int retval = 0;

  retval = libusb_bulk_transfer(device_handle, endpoint_out, buf, size, &nActual, timeout);
  if (retval)
  {
    fprintf(stderr, "USB write failed with code %d.\n\n", retval);
  }
  else if (nActual != size)
  {
    size = nActual;
    retval = 1;
    fprintf(stderr, "write count wrong.\n\n");
  }
  else if ((nActual % endpoint_out_max_packet_size) == 0)
  {
    fprintf(stderr, "Device write, size larger than max packet size.\n\n");
    retval = libusb_bulk_transfer(device_handle, endpoint_out, nullptr, 0, &nActual, timeout);
  }
  return retval;
}

int Polhemus::device_init(void)
{
  if (libusb_set_configuration(device_handle, CONFIGURATION) != 0)
    fprintf(stderr, "Could not set usb configuration to %d\n", CONFIGURATION);

  if (libusb_claim_interface(device_handle, INTERFACE) != 0)
  {
    fprintf(stderr, "Could not claim usb interface %d\n", INTERFACE);
    return 1;
  }

  device_reset();
  return 0;
}

int Polhemus::device_send(uint8_t *cmd, int &count)
{
  if (device_write(cmd, count, TIMEOUT))
  {
    warn("Sending cmd `%s' to device failed\n");
    return 1;
  }
  return 0;
}

int Polhemus::device_read(void *pbuf, int &size, bool bTOisErr)
{
  uint32_t timeout = VPUSB_READ_TIMEOUT_MS;
  int retval = 0;
  int nActual = 0;

  retval = libusb_bulk_transfer(device_handle, endpoint_in, (unsigned char*)pbuf, size, &nActual, timeout);

  if ((retval == LIBUSB_ERROR_TIMEOUT) && bTOisErr)
  {
    retval = 1;
    size = 0;
  } else
  {
    size = nActual;
  }

  return retval;
}

void Polhemus::device_clear_input(void)
{
  g_nrxcount = rx_buffer_size;

  while(g_nrxcount > 0)
  {
    device_read(g_rxbuf, g_nrxcount, true);
  }
}

int Polhemus::send_saved_calibration(void)
{
  int retval = 1;

  retval = receive_pno_data_frame();

  device_reset();

  reset_boresight();

  // send the calibration saved in calibration.yaml
  // read from param server the x, y and z for all stations, so we need to loop stations and send boresight command
  for (int i = 0; i < station_count; ++i)
  {
    if (nh->hasParam("/calibration/viper_calibration/rotations/station_" + std::to_string(i)))
    {
      std::string x_name = "/calibration/viper_calibration/rotations/station_" + std::to_string(i) + "/x";
      std::string y_name = "/calibration/viper_calibration/rotations/station_" + std::to_string(i) + "/y";
      std::string z_name = "/calibration/viper_calibration/rotations/station_" + std::to_string(i) + "/z";
      float x;
      float y;
      float z;

      // read x y and z rotations from param server
      nh->getParam(x_name, x);
      nh->getParam(y_name, y);
      nh->getParam(z_name, z);

      if (x == 0 & y == 0 & z == 0)
      {
        // no previous calibration exists
        fprintf(stderr, "No previous calibration data available, please calibrate before proceeding!!!\n\n");
        break;
      }
      else
      {
        fprintf(stderr, "Calibrating station %d.\n", i);

        if (name == "viper")
        {
          retval = define_data_type(DATA_TYPE_EULER);
        }
        else
        {
          retval = 0;
        }
        if (!retval)
        {
          tf2::Quaternion q = get_quaternion(i);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

          fprintf(stderr, "quat x: %f\n", q[0]);
          fprintf(stderr, "quat y: %f\n", q[1]);
          fprintf(stderr, "quat z: %f\n", q[2]);
          fprintf(stderr, "quat w: %f\n", q[3]);

          fprintf(stderr, "roll %f.\n", roll);
          fprintf(stderr, "pitch %f.\n", pitch);
          fprintf(stderr, "yaw %f.\n", yaw);

          roll = (roll * 180) / PI;
          pitch = (pitch * 180) / PI;
          yaw = (yaw * 180) / PI;

          fprintf(stderr, "roll %f.\n", roll);
          fprintf(stderr, "pitch %f.\n", pitch);
          fprintf(stderr, "yaw %f.\n", yaw);

          x = roll - x;
          y = pitch - y;
          z = yaw - z;

          fprintf(stderr, "x %f.\n", x);
          fprintf(stderr, "y %f.\n", y);
          fprintf(stderr, "z %f.\n", z);

          retval = set_boresight(false, i, z, y, x);
          if (retval)
          {
            fprintf(stderr, "Error sending calibration from file.\n\n");
            break;
          }
        }
      }
    }
    else
    {
      // no previous calibration exists
      fprintf(stderr, "Station could not be found in calibration, please calibrate before proceeding!!!\n\n");
      break;
    }
  }
  define_data_type(DATA_TYPE_QUAT);
  device_data_mode(DATA_CONTINUOUS);
}

bool Polhemus::calibrate(void)
{
  int retval = 1;

  if (name == "viper")
  {
    device_data_mode(DATA_RESET);
  }
  else
  {
    device_data_mode(DATA_SINGLE);
  }

  // turn off continuous data mode first

  device_clear_input();
  reset_boresight();

  define_data_type(DATA_TYPE_EULER);

  for (int i = 0; i < station_count; ++i)
  {
    tf2::Quaternion q = get_quaternion(i);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    fprintf(stderr, "quat x: %f\n", q[0]);
    fprintf(stderr, "quat y: %f\n", q[1]);
    fprintf(stderr, "quat z: %f\n", q[2]);
    fprintf(stderr, "quat w: %f\n", q[3]);

    // convert to degrees
    roll = (roll * 180) / 3.14;
    pitch = (pitch * 180) / 3.14;
    yaw = (yaw * 180) / 3.14;

    fprintf(stderr, "roll: %f\n", roll);
    fprintf(stderr, "pitch: %f\n", pitch);
    fprintf(stderr, "yaw: %f\n", yaw);

    fprintf(stderr, "Calibrating station %d.\n", i);

    // save values to config file
    std::string x_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/x";
    std::string y_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/y";
    std::string z_name = "/calibration/" + name + "_calibration/rotations/station_" + std::to_string(i) + "/z";

    nh->setParam(x_name, roll);
    nh->setParam(y_name, pitch);
    nh->setParam(z_name, yaw);

    if (name == "viper")
    {
      system(" echo 'Calibration file saved at: ' $(rospack find polhemus_ros_driver)/config/; rosparam dump $(rospack find polhemus_ros_driver)/config/viper_calibration.yaml /calibration");
    }
    else
    {
      system(" echo 'Calibration file saved at: ' $(rospack find polhemus_ros_driver)/config/; rosparam dump $(rospack find polhemus_ros_driver)/config/liberty_calibration.yaml /calibration");
    }
  }

  retval = set_boresight(false, -1, 0, 0, 0);
  if (retval)
  {
    fprintf(stderr, "Calibration failed.\n");
  }

  // set data mode back to continuous
  retval = define_data_type(DATA_TYPE_QUAT);
  if (retval)
  {
    fprintf(stderr, "quat failed.\n");
  }

  retval = device_data_mode(DATA_CONTINUOUS);
  if (retval)
  {
    fprintf(stderr, "continuous failed.\n");
  }

  return true;
}

bool Polhemus::calibrate_srv(polhemus_ros_driver::calibrate::Request &req, polhemus_ros_driver::calibrate::Response &res)
{
  printf("Calibration request...");
  res.success = calibrate();
  return true;
}

bool Polhemus::persist_srv(polhemus_ros_driver::persist::Request &req, polhemus_ros_driver::persist::Response &res)
{
  printf("Making config persistent");
  res.success = persist_commands();
  return true;
}

int Polhemus::device_reset(void)
{
}

int Polhemus::request_num_of_stations(void)
{
}

int Polhemus::set_hemisphere(int x, int y, int z)
{
}

int Polhemus::define_data_type(data_type_e data_type)
{
}

void Polhemus::device_binary_mode(void)
{
}

void Polhemus::generate_data_structure(void)
{
}

int Polhemus::receive_pno_data_frame(void)
{
}

int Polhemus::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
{
}

int Polhemus::device_data_mode(data_mode_e mode)
{
}

int Polhemus::set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4)
{
}

int Polhemus::reset_boresight(void)
{
}

bool Polhemus::persist_commands(void)
{
}

int Polhemus::set_source(int source)
{
}

tf2::Quaternion Polhemus::get_quaternion(int station_id)
{
}

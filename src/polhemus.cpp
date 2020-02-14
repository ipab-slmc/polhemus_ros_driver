
/*

 Communication library for a Polhemus Liberty v2 (tm) Motion tracker
 Copyright (C) 2008 Jonathan Kleinehellefort <kleinehe@cs.tum.edu>
     Intelligent Autonomous Systems Lab,
     Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen

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



#include <polhemus_ros_driver/polhemus.hpp>
#include <string.h>
#include <libusb-1.0/libusb.h>


#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif

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

void Polhemus::init_buffer(void)
{
  buffer_in.fill = 0;
}

int Polhemus::device_write(uint8_t *buf, int size, int timeout)
{
  int nActual = 0;
  int r = 0;
  r = libusb_bulk_transfer(device_handle, endpoint_out, buf, size, &nActual, timeout);
  fprintf(stderr, "return code %d.\n\n", r);
  if (r)
  {
    //r = (r << 16) | E_VPERR_LIBUSB
  }
  else if (nActual != size)
  {
    size = nActual;
    r = -1;// E_VPUSB_ERR_WRITE_COUNT_WRONG;
    fprintf(stderr, "write count wrong.\n\n");
  }
  else if ((nActual % MAX_PACKET_SIZE) == 0)
  {
    fprintf(stderr, "larger than max packet size.\n\n");
    r = libusb_bulk_transfer(device_handle, endpoint_out, nullptr, 0, &nActual, timeout);
  }
  return r;
}

int Polhemus::device_init(void)
{
  if (libusb_set_configuration(device_handle, CONFIGURATION) != 0)
    fprintf(stderr, "could not set usb configuration to %d\n", CONFIGURATION);

  if (libusb_claim_interface(device_handle, INTERFACE) != 0)
  {
    fprintf(stderr, "could not claim usb interface %d\n", INTERFACE);
    return 0;
  }

  /*
   static char magic[] = { '*', '*', 0xff, 0x16, 0, 0, 0, 0 };
   if (liberty_write(handle, magic, sizeof(magic), 0) != sizeof(magic)) {
   warn("usb bulk write failed\n");
   return 0;
   }*/
  device_reset();
  return 1;
}

int Polhemus::device_send(uint8_t *cmd, int &count)
{
  if (device_write(cmd, count, TIMEOUT) != count)
  {
    warn("sending cmd `%s' to device failed\n");
    return 0;
  }
  return 1;
}

int Polhemus::device_read(uint8_t *pbuf, int &size, bool bTOisErr)
{
  uint32_t timeout = VPUSB_READ_TIMEOUT_MS;
  int retval = 0;
  int nActual = 0;

  retval = libusb_bulk_transfer(device_handle, endpoint_in, pbuf, size, &nActual, timeout);
  if ((retval == LIBUSB_ERROR_TIMEOUT) && !bTOisErr)
  {
    retval = 0;
    size = 0;
  } else
  {
    size = nActual;
  }

  return retval;
}

void Polhemus::device_clear_input(void)
{
//  uint8_t buf[1024];
//
//    while(device_read(buf, sizeof(buf), TIMEOUT) > 0);
}

void Polhemus::device_ignore_input(int count)
{
//    uint8_t buf[2048];
//    int i;
//    for (i = 0; i < count; ++i)
//        device_read(buf, sizeof(buf), TIMEOUT);
}

int Polhemus::device_receive(void *buf, int size)
{
//  //printf("liberty_receive: %u\n",size);
//  uint8_t bufff[1024];
//  int test;
//  while (1)
//  {
//
//    while (buffer_in.fill < size)
//    {
//      int n_read = device_read(bufff, test, true);
//      warn("read %d\n", n_read);
//      if (n_read < 0)
//      {
//        warn("error while reading from device (%d)", n_read);
//        return 0;
//      }
//      buffer_in.fill += n_read;
//    }
//
//#ifdef DEBUG
//    fprintf(stderr, "- %c%c\n", b->buf[0], b->buf[1]);
//#endif
//
//    if (buffer_in.buf[0] == 'L' && buffer_in.buf[1] == 'Y')
//    {
//      memcpy(buf, buffer_in.buf, size);
//      memmove(buffer_in.buf, buffer_in.buf + size, buffer_in.fill - size);
//      buffer_in.fill -= size;
//      return 1;
//    } else
//    {
//      warn("got corrupted data\n");
//      buffer_in.fill = 0;
//    }
//  }
}



/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */
void Polhemus::device_reset(void)
{
}

int Polhemus::request_num_of_stations(void)
{
	return 0;
}

int Polhemus::set_hemisphere(int x, int y, int z)
{
}

int Polhemus::define_quat_data_type(void)
{
}

void Polhemus::device_binary_mode(void)
{
}

void Polhemus::generate_data_structure(void)
{
}

int Polhemus::receive_pno_data(void)
{
}

void Polhemus::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
{
}

int Polhemus::device_data_mode(data_mode_e mode)
{
}

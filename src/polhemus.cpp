/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
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
  int retval = 0;
  retval = libusb_bulk_transfer(device_handle, endpoint_out, buf, size, &nActual, timeout);
  if (retval)
  {
    //r = (r << 16) | E_VPERR_LIBUSB
  }
  else if (nActual != size)
  {
    size = nActual;
    retval = -1;// E_VPUSB_ERR_WRITE_COUNT_WRONG;
    fprintf(stderr, "write count wrong.\n\n");
  }
  else if ((nActual % MAX_PACKET_SIZE) == 0)
  {
    fprintf(stderr, "larger than max packet size.\n\n");
    retval = libusb_bulk_transfer(device_handle, endpoint_out, nullptr, 0, &nActual, timeout);
  }
  return retval;
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

int Polhemus::device_read(void *pbuf, int &size, bool bTOisErr)
{
  uint32_t timeout = VPUSB_READ_TIMEOUT_MS;
  int retval = 0;
  int nActual = 0;

  retval = libusb_bulk_transfer(device_handle, endpoint_in, (unsigned char*)pbuf, size, &nActual, timeout);
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
  uint8_t buf[1024];
  g_nrxcount = RX_BUF_SIZE;

  while(g_nrxcount > 0)
  {
    device_read(g_rxbuf, g_nrxcount, true);
  }
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


int Polhemus::device_reset(void)
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

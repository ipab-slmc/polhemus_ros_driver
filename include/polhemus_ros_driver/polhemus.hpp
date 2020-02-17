/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef POLHEMUS_H
#define POLHEMUS_H

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>


#define INTERFACE 0
#define CONFIGURATION 1
#define TIMEOUT 1000
#define CRC_BYTES 4
#define SEUID 0
#define MAX_PACKET_SIZE 64
#define TX_BUF_SIZE  0x400
#define RX_BUF_SIZE  0x400
#define VPUSB_WRITE_TIMEOUT_MS 50
#define VPUSB_READ_TIMEOUT_MS 300

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

typedef struct buffert_t {
    uint8_t buf[8192];
    int fill;
} buffer_t;

typedef enum data_mode_e {
    DATA_CONTINUOUS,
    DATA_SINGLE,
    DATA_RESET
} data_mode_e;

class Polhemus {
public:
	virtual ~Polhemus(void);
	int count_bits(uint16_t v);
	void init_buffer(void);

	/* set up usb interface and configuration, send initial magic and reset */
	int device_init(void);
	virtual void device_binary_mode(void);
	virtual int device_data_mode(data_mode_e mode);
	/* send a command */
	int device_send(uint8_t *cmd, int &count);
	/* read until the device doesn't send anything else */
	void device_clear_input(void);
	void device_ignore_input(int count);
	virtual int receive_pno_data(void);
	virtual void fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id);

	int device_read(void *pbuf, int &count, bool bTOisErr/*=false*/);
	int device_write(uint8_t *buf, int size, int timeout);
	virtual int device_reset(void);
	virtual int define_quat_data_type(void);
	virtual void generate_data_structure(void);
	virtual int set_hemisphere(int x, int y, int z);
	virtual int request_num_of_stations(void);
	int device_receive(void *buf, int size);
	libusb_device_handle *device_handle;
  int station_count;
  uint8_t endpoint_in;
  uint8_t endpoint_out;
  int g_ntxcount;
  int g_nrxcount;
  uint8_t g_txbuf[TX_BUF_SIZE];
  uint8_t g_rxbuf[RX_BUF_SIZE];
private:
  buffer_t buffer_in;

};
#endif

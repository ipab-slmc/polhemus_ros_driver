/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef VIPER_H
#define VIPER_H

#include <polhemus_ros_driver/polhemus.hpp>
#include <polhemus_ros_driver/viper_protocol.h>
#include <libusb-1.0/libusb.h>


#define VIPER_ENDPOINT_IN 0x81
#define VIPER_ENDPOINT_OUT 0x02
#define VIPER_TX_BUF_SIZE  0x400
#define VIPER_RX_BUF_SIZE  0x400

class Viper: public Polhemus
{
public:
  Viper(void);
  ~Viper(void);
  void device_clear_input(void);
  int request_num_of_stations(void);
  int device_reset(void);
  int device_data_mode(data_mode_e mode);
  int receive_pno_data_frame(void);
  int fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id);
  int define_quat_data_type(void);
  int set_hemisphere(int x, int y, int z);
private:
  uint32_t calc_crc_bytes(uint8_t *data, uint32_t count);
  void crc_16(uint32_t * crc, uint32_t data);
  viper_full_header_t* fill_command(uint32_t cmd, uint32_t act, uint32_t arg1 = 0, uint32_t arg2 = 0, void *payload = 0,
                                    uint32_t payload_size = 0);
  void prepare_frame(uint8_t buffer[], int &txbytes);
  int receive_data_frame(viper_cmds_e cmd_type);
  int set_boresight(bool reset_origin, int arg_1, int arg_2, int arg_3, int arg_4 = 0);
  int set_source(int source);
  bool calibrate(void);
  bool persist_commands(void);
  CVPSeuPno pno;
  uint32_t sensor_map;
  uint8_t g_rxbuf[VIPER_RX_BUF_SIZE];
  uint8_t g_txbuf[VIPER_TX_BUF_SIZE];
};
#endif

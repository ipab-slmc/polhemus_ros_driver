/*
* Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <polhemus_ros_driver/viper.hpp>
#include <ros/console.h>

#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif

#define VALIDATE_CONTEXT(pctx, ctx) {pctx=(CVPcontext*)ctx;if (!(CVPcontext::findPctx(pctx))) return E_VPERR_INVALID_CONTEXT; }


Viper::Viper(void) : Polhemus()
{
}
Viper::~Viper(void) {}

int Viper::device_reset(void)
{
  int retval = device_data_mode(DATA_RESET);
  device_clear_input();
  return retval;
}


int Viper::device_data_mode(data_mode_e mode)
{
  viper_cmds_e cmd_type;
  viper_cmd_actions_e action;
  CVPcmd viper_command;
  switch (mode)
  {
    case DATA_CONTINUOUS:
      cmd_type = CMD_CONTINUOUS_PNO;
      action = CMD_ACTION_SET;
      break;
    case DATA_SINGLE:
      cmd_type = CMD_SINGLE_PNO;
      action = CMD_ACTION_SET;
      break;
    case DATA_RESET:
      cmd_type = CMD_CONTINUOUS_PNO;
      action = CMD_ACTION_RESET;
      break;
    default:
      return -1;
  }
  viper_command.Fill(cmd_type, action);
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  int retval = device_send(pbuf, nBytes);
  if (retval)
  {
    fprintf(stderr, "retval %d.\n\n", retval);
  }
  else
  {
    retval = receive_data_frame(cmd_type);
  }
  return retval;

}

int Viper::receive_data_frame(viper_cmds_e cmd_type)
{
  int retval = 0;
  g_nrxcount = RX_BUF_SIZE;
  retval = device_read(g_rxbuf, g_nrxcount, true);

  if (retval == 0)
  {

    CFrameInfo fi(g_rxbuf, g_nrxcount);
    if ((fi.cmd() != cmd_type) || !(fi.IsAck()))
    {
      retval = -1;
    }
  }
  return retval;
}

int Viper::receive_pno_data_frame(void)
{
  int retval = 0;
  g_nrxcount = RX_BUF_SIZE;
  retval = device_read(g_rxbuf, g_nrxcount, true);

  if (retval == 0)
  {
    CFrameInfo fi(g_rxbuf, g_nrxcount);

    uint32_t bytesextracted;
    bytesextracted = pno.Extractseupno(fi.PPnoBody());

    if (!bytesextracted)
    {
      retval = -1;
    }
  }
  return retval;
}

void Viper::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
{
  // Set translation (in metres)

  transform->transform.translation.x = pno.SensFrame(station_id)->pno.pos[0];
  transform->transform.translation.y = pno.SensFrame(station_id)->pno.pos[1];
  transform->transform.translation.z = pno.SensFrame(station_id)->pno.pos[1];
  // Set rotation
  transform->transform.rotation.w = pno.SensFrame(station_id)->pno.ori[0];
  transform->transform.rotation.x = pno.SensFrame(station_id)->pno.ori[1];
  transform->transform.rotation.y = pno.SensFrame(station_id)->pno.ori[2];
  transform->transform.rotation.z = pno.SensFrame(station_id)->pno.ori[3];
}

int Viper::define_quat_data_type(void)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_UNITS;
  viper_cmd_actions_e action = CMD_ACTION_SET;

  viper_units_config_t cfg;
  cfg.ori_units = ORI_QUATERNION;
  cfg.pos_units = POS_METER;
  CVPcmd viper_command;
  viper_command.Fill(cmd_type, action, -1, 0, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  retval = device_send(pbuf, nBytes);
  if (retval)
  {
    ;
  }
  else
  {
    retval = receive_data_frame(cmd_type);
  }

  return retval;
}

int Viper::request_num_of_stations(void)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_STATION_MAP;
  viper_cmd_actions_e action = CMD_ACTION_GET;
  CVPcmd viper_command;

  CStationMap cstamap;
  viper_command.Fill(cmd_type, action);

  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;

  retval = device_send(pbuf, nBytes);

  if (retval)
  {
    ;
  }
  else
  {
    g_nrxcount = RX_BUF_SIZE;
    retval = device_read(g_rxbuf, g_nrxcount, true);

    if (retval == 0)
    {
      CFrameInfo fi(g_rxbuf, g_nrxcount);

      if (fi.cmd() == CMD_STATION_MAP)
      {
        CStationMap cmap((viper_station_map_t*) fi.PCmdPayload());
        cstamap = cmap;

      }
    }
  }

  station_count = cstamap.SnsDetectedCount();
  return retval;
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
int Viper::set_hemisphere(int x, int y, int z)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_HEMISPHERE;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_hemisphere_config_t cfg;
  cfg.track_enabled = 1;
  cfg.params[0] = x;
  cfg.params[1] = y;
  cfg.params[2] = z;
  CVPcmd viper_command;
  viper_command.Fill(cmd_type, action, -1, 0, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  retval = device_send(pbuf, nBytes);
  if (retval)
  {
    ;
  }
  else
  {
    retval = receive_data_frame(cmd_type);
  }
  return retval;
}

int Viper::set_boresight(bool reset_origin, int arg_1, int arg_2, int arg_3, int arg_4)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_BORESIGHT;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_boresight_config_t cfg;
  cfg.params[0] = arg_1;
  cfg.params[1] = arg_2;
  cfg.params[2] = arg_3;
  cfg.params[3] = arg_4;
  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action, -1, reset_origin, &cfg, sizeof(cfg));
  viper_command.Prepare(g_txbuf, g_ntxcount);

  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  retval = device_send(pbuf, nBytes);
  if (retval)
  {
    ;
  }
  else
  {
    retval = receive_data_frame(cmd_type);
  }
  return retval;
}

bool Viper::calibrate(void)
{
  set_boresight(false, 1, 0, 0, 0);

  return true;
}



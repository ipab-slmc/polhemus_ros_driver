
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


#include <polhemus_ros_driver/viper.hpp>

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
  endpoint_in = VIPER_ENDPOINT_IN;
  endpoint_out = VIPER_ENDPOINT_OUT;
}
Viper::~Viper(void) {}

/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */
void Viper::device_reset(void)
{
  ;
}

void Viper::device_binary_mode(void)
{
  // viper is binary mode only
  ;
}

void Viper::generate_data_structure(void)
{
  stations = (viper_pno_frame_t*) malloc(sizeof(viper_pno_frame_t) + (sizeof(viper_pno_data_t) * station_count));

}

void Viper::device_data_mode(data_mode_e mode)
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
      ;
  }
  viper_command.Fill(cmd_type, action);
  viper_command.Prepare(g_txbuf, g_ntxcount);
  int nBytes = g_ntxcount;
  uint8_t *pbuf = g_txbuf;
  device_send(pbuf, nBytes);
}

void Viper::receive_pno_data(void)
{
  if (!device_receive(stations, sizeof(viper_pno_header_t) + (sizeof(viper_pno_data_t) * station_count)) + CRC_BYTES) {
    fprintf(stderr, "Receive failed.\n");
  }
}

void Viper::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
{
  // Set translation (in metres)
  transform->transform.translation.x = stations->pno_data[station_id].pos[0];
  transform->transform.translation.y = stations->pno_data[station_id].pos[1];
  transform->transform.translation.z = stations->pno_data[station_id].pos[2];

  // Set rotation
  transform->transform.rotation.w = stations->pno_data[station_id].ori[0];
  transform->transform.rotation.x = stations->pno_data[station_id].ori[1];
  transform->transform.rotation.y = stations->pno_data[station_id].ori[2];
  transform->transform.rotation.z = stations->pno_data[station_id].ori[3];
}

void Viper::define_quat_data_type(void)
{
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
  device_send(pbuf, nBytes);
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
    for (std::size_t i = 0; i < g_nrxcount; i++)
    {
      std::cout << int(g_rxbuf[i]) << ' ';
    }
    if (retval == 0)
    {
      CFrameInfo fi(g_rxbuf, g_nrxcount);
      fprintf(stderr, "cmd %d .\n\n", fi.cmd());
      if ((fi.cmd() == CMD_STATION_MAP) && (fi.IsAck()))
      {
        fprintf(stderr, "here innit.\n\n");
        CStationMap cmap((viper_station_map_t*) fi.PCmdPayload());
        cstamap = cmap;

      }
    }
  }
  int station_count = cstamap.SnsDetectedCount();
  fprintf(stderr, "station count %d.\n\n", station_count);

  return station_count;
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
void Viper::set_hemisphere(int x, int y, int z)
{
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
  device_send(pbuf, nBytes);
}



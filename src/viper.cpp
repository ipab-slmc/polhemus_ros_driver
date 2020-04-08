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
      fprintf(stderr, "cmd: %d\n", fi.cmd());
      fprintf(stderr, "action: %d\n", fi.action());
      fprintf(stderr, "cmd sent: %d\n", cmd_type);
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
      return retval;
    }
    // bitmap of active sensors
    sensor_map = pno.SensorMap();

    // update this as a sensor may have been dropped
    station_count = pno.SensorCount();
    return station_count;
  }
  return retval;
}

int Viper::fill_pno_data(geometry_msgs::TransformStamped *transform, int count)
{
  // Set translation (in metres)
  int retval = 0;
  transform->child_frame_id = "polhemus_station_" + std::to_string(pno.SensFrame(count)->SFinfo.bfSnum);
  transform->transform.translation.x = pno.SensFrame(count)->pno.pos[0];
  transform->transform.translation.y = pno.SensFrame(count)->pno.pos[1];
  transform->transform.translation.z = pno.SensFrame(count)->pno.pos[2];
  // Set rotation
  transform->transform.rotation.w = pno.SensFrame(count)->pno.ori[0];
  transform->transform.rotation.x = pno.SensFrame(count)->pno.ori[1];
  transform->transform.rotation.y = pno.SensFrame(count)->pno.ori[2];
  transform->transform.rotation.z = pno.SensFrame(count)->pno.ori[3];

  return retval;
}

int Viper::define_data_type(data_type_e data_type)
{
  int retval = 0;

  viper_cmds_e cmd_type = CMD_UNITS;
  viper_cmd_actions_e action = CMD_ACTION_SET;

  viper_units_config_t cfg;

  if (data_type == DATA_TYPE_QUAT)
  {
    cfg.ori_units = ORI_QUATERNION;
  }
  else if (data_type == DATA_TYPE_EULER)
  {
    cfg.ori_units = ORI_EULER_DEGREE;
  }
  else
  {
    return 1;
  }

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
  cfg.track_enabled = 0;
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

int Viper::set_boresight(bool reset_origin, int station, float arg_1, float arg_2, float arg_3, float arg_4)
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

  viper_command.Fill(cmd_type, action, station, reset_origin, &cfg, sizeof(cfg));
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

int Viper::set_source(int source)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_SRC_SELECT;
  viper_cmd_actions_e action = CMD_ACTION_SET;
  viper_src_select_cfg_t cfg;
  cfg.src_select_map = 0;
  CVPcmd viper_command;

  viper_command.Fill(cmd_type, action, 0, 0, &cfg, sizeof(cfg));
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

int Viper::send_saved_calibration(float x, float y, float z, int station_id)
{
  int retval = 1;

  // set data type to euler first
  int result = define_data_type(DATA_TYPE_EULER);
  if (!result)
  {
    set_boresight(false, station_id, x, y, z);
    retval = 0;
  }
  return retval;

}

bool Viper::calibrate(void)
{
  for (int i = 0; i < pno.SensorCount(); ++i)
  {
    tf2::Quaternion q(
        pno.SensFrame(i)->pno.ori[1],
        pno.SensFrame(i)->pno.ori[2],
        pno.SensFrame(i)->pno.ori[3],
        pno.SensFrame(i)->pno.ori[0]);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    fprintf(stderr, "quat x: %f\n", pno.SensFrame(i)->pno.ori[1]);
    fprintf(stderr, "quat y: %f\n", pno.SensFrame(i)->pno.ori[2]);
    fprintf(stderr, "quat z: %f\n", pno.SensFrame(i)->pno.ori[3]);
    fprintf(stderr, "quat w: %f\n", pno.SensFrame(i)->pno.ori[0]);

    // convert to degrees
    roll = (roll * 180) / 3.14;
    pitch = (pitch * 180) / 3.14;
    yaw = (yaw * 180) / 3.14;

    fprintf(stderr, "roll: %f\n", roll);
    fprintf(stderr, "pitch: %f\n", pitch);
    fprintf(stderr, "yaw: %f\n", yaw);

    int result = define_data_type(DATA_TYPE_EULER);
    if (!result)
    {
      fprintf(stderr, "Calibrating station %d.\n", i);
      //set_boresight(false, i, 1, 0, 0, 0);
      set_boresight(false, i, -roll, -pitch, -yaw);
    }

    result = define_data_type(DATA_TYPE_QUAT);

    // save values to config file
    std::string x_name = "/viper_calibration/rotations/station_" + std::to_string(i) + "/x";
    std::string y_name = "/viper_calibration/rotations/station_" + std::to_string(i) + "/y";
    std::string z_name = "/viper_calibration/rotations/station_" + std::to_string(i) + "/z";

    double x, y, z;

    x = - roll;
    y = - pitch;
    z = - yaw;

    nh->setParam(x_name, roll);
    nh->setParam(y_name, pitch);
    nh->setParam(z_name, yaw);
    system(" echo 'Calibration file saved at: ' $(rospack find polhemus_ros_driver)/config/; rosparam dump $(rospack find polhemus_ros_driver)/config/viper_calibration.yaml /viper_calibration");
  }

  // YAML::Node node, _baseNode = YAML::LoadFile("cfg/calibration.yaml");

  return true;
}

bool Viper::persist_commands(void)
{
  int retval = 0;
  viper_cmds_e cmd_type = CMD_PERSIST;
  viper_cmd_actions_e action = CMD_ACTION_SET;

  CVPcmd viper_command;

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
    retval = receive_data_frame(cmd_type);
  }

  if (! retval)
  {
    return true;
  }
  else
  {
    return false;
  }
}


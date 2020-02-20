
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


#include <polhemus_ros_driver/liberty.hpp>

#ifdef DEBUG
#include <stdio.h>
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, as); }
#else
#define warn(as...)
#endif


Liberty::Liberty(void) : Polhemus()
{
}
Liberty::~Liberty(void) {}

/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */
int Liberty::device_reset(void)
{
  // reset c, this may produce "invalid command" answers
  unsigned char command[] = "\rp\r";
  int size = sizeof(command);
  device_send(command, size);
  // remove everything from input
  device_clear_input();
}

void Liberty::device_binary_mode(void)
{
  unsigned char command[] = "f1\r";
  int size = sizeof(command);
  device_send(command, size);
}

void Liberty::generate_data_structure(void)
{
  stations = (liberty_pno_frame_t*) (malloc(sizeof(liberty_pno_frame_t) * station_count));
}

int Liberty::device_data_mode(data_mode_e mode)
{

  int size;
  switch (mode)
  {
    case DATA_CONTINUOUS:
    {
      unsigned char command[] = "c\r";
      size = sizeof(command);
      device_send(command, size);
      return 0;
    }
    case DATA_SINGLE:
    {
      unsigned char command[] = "p";
      size = sizeof(command);
      device_send(command, size);
      return 0;
    }
    default:
      return 0;
  }
}

int Liberty::receive_pno_data_frame(void)
{
  int retval = 0;
  g_nrxcount = sizeof(liberty_pno_frame_t) * station_count;
  retval = device_read(stations, g_nrxcount, true);
  if (retval)
  {
    fprintf(stderr, "Receive failed.\n");
  }
}

int Liberty::fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id)
{
  // Set translation (conversion: inches -> meters)
  transform->transform.translation.x = 0.0254*stations[station_id].x;
  transform->transform.translation.y = 0.0254*stations[station_id].y;
  transform->transform.translation.z = 0.0254*stations[station_id].z;

  // Set rotation
  transform->transform.rotation.w = stations[station_id].quaternion[0];
  transform->transform.rotation.x = stations[station_id].quaternion[1];
  transform->transform.rotation.y = stations[station_id].quaternion[2];
  transform->transform.rotation.z = stations[station_id].quaternion[3];
}

int Liberty::define_quat_data_type(void)
{
  unsigned char command[] = "O*,8,9,11,3,7\r";  // quaternions
  int size = sizeof(command);
  device_send(command, size);
  return 0;
}

int Liberty::request_num_of_stations(void)
{
  int retval = 0;
  unsigned char command[] = { control('u'), '0', '\r', '\0' };
  active_station_state_response_t resp;
  int size = sizeof(command);
  int size_reply = sizeof(resp);
  device_send(command, size);
  retval = device_read(&resp, size_reply, true);
  g_nrxcount = RX_BUF_SIZE;

  retval = device_read(g_rxbuf, g_nrxcount, true);

  if (resp.head.init_cmd == 21) {
    station_count = count_bits(resp.detected & resp.active);
    return station_count;
  }
  else {
    fprintf(stderr, "init command returned: %d %d %d \n", resp.head.init_cmd, resp.head.station, resp.head.error);
    return 0;
  }
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
int Liberty::set_hemisphere(int x, int y, int z)
{
  unsigned char command[32];
  int size = sizeof(command);
  snprintf((char *)command, size, "h*,%u,%u,%u\r", x, y, z);
  device_send(command, size);
  return true;
}

bool Liberty::calibrate(void)
{
  unsigned char command[] = "b*,0,0,0,0\r";
  int size = sizeof(command);
  device_send(command, size);
  return true;
}


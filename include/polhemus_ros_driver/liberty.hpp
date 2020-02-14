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


#ifndef LIBERTY_H
#define LIBERTY_H

#include <polhemus_ros_driver/polhemus.hpp>
#include "polhemus_ros_driver/liberty_protocol.h"
#include <libusb-1.0/libusb.h>


#define LIBERTY_ENDPOINT_IN 0x88
#define LIBERTY_ENDPOINT_OUT 0x4

class Liberty : public Polhemus {
public:
  Liberty(void);
	~Liberty(void);
	int request_num_of_stations(void);
	int device_reset(void);
	void device_binary_mode(void);
	int device_data_mode(data_mode_e mode);
	int receive_pno_data(void);
	void fill_pno_data(geometry_msgs::TransformStamped *transform, int station_id);
	void generate_data_structure(void);
	int define_quat_data_type(void);
	int set_hemisphere(int x, int y, int z);
private:
	liberty_pno_frame_t *stations;
};
#endif

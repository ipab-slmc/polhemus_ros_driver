/*

 Communication library for a Polhemus Liberty v2 (tm) Motion tracker
 Copyright (C) 2008 Jonathan Kleinehellefort and Alexis Maldonado
 Intelligent Autonomous Systems Lab,
 Lehrstuhl fuer Informatik 9, Technische Universitaet Muenchen
 <kleinehe@cs.tum.edu>  <maldonad@cs.tum.edu>

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

#include <usb.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>

#include <sys/time.h>
#include <time.h>
#include "liberty.h"
#include "protocol.h"
#include <ros/ros.h>

#include "polhemus_ros/polhemus_ros.h"
/* Vendor 0x0f44 -> Polhemus */
#define VENDOR 0xf44
/* Product 0xff20 -> Liberty v2 Motion Tracker (with USB 2.0 using an EzUSB fx2
 chip)  after loading the firmware (it has ProductId 0xff21 before) */
#define PRODUCT 0xff20

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

static int count_bits(uint16_t v)
{
	int c;
	for (c = 0; v; c++)
	{
		v &= v - 1; // clear the least significant bit set
	}
	return c;
}

/* main loop running? */
static int go_on;

static void signal_handler(int s)
{
	switch (s)
	{
		case SIGINT:
			go_on = 0;
			break;
	}
}

static void print_hex(FILE *stream, const char *buf, size_t size)
{
	const char *c;
	for (c = buf; c != buf + size; ++c)
		fprintf(stream, "%02x:%c ", (unsigned char) *c, isprint(*c) ? *c : '.');
	fprintf(stream, "\n");
}

static void print_ascii(FILE *stream, const char *buf, size_t size)
{
	const char *c;
	for (c = buf; c != buf + size; ++c)
		if (isprint(*c))
		{
			fprintf(stream, "%c", *c);
		}
	fprintf(stream, "\n");
}

static struct usb_device *find_device_by_id(uint16_t vendor, uint16_t product)
{
	struct usb_bus *bus;

	usb_find_busses();
	usb_find_devices();

	for (bus = usb_get_busses(); bus; bus = bus->next)
	{
		struct usb_device *dev;
		for (dev = bus->devices; dev; dev = dev->next)
		{
			if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
				return dev;
		}
	}
	return NULL;
}

static int request_num_of_stations(usb_dev_handle *handle, buffer_t *b)
{
	static char cmd[] = { control('u'), '0', '\r', '\0' };
	active_station_state_response_t resp;
	liberty_send(handle, cmd);
	liberty_receive(handle, b, &resp, sizeof(resp));

	//printf("station number: 0x%0x\n",resp.head.station);
	//printf("Init cmd: 0x%0x\n",resp.head.init_cmd);
	//printf("Error: 0x%0x\n",resp.head.error);
	//printf("Reserved: 0x%0x\n",resp.head.reserved);
	if (resp.head.init_cmd == 21)
	{
		return count_bits(resp.detected & resp.active);
	}
	else
	{
		return 0;
	}
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
static void set_hemisphere(usb_dev_handle *handle, int x, int y, int z)
{
	char cmd[32];
	snprintf(cmd, sizeof(cmd), "h*,%d,%d,%d\r", x, y, z);
	liberty_send(handle, cmd);
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "PolhemusLiberty");

	int i;
	struct usb_device *dev;
	usb_dev_handle *handle;

	usb_init();

	dev = find_device_by_id(VENDOR, PRODUCT);
	if (!dev)
	{
		fprintf(stderr, "Could not find the Polhemus Liberty Device.\n");
		abort();
	}

	handle = usb_open(dev);
	if (!handle)
	{
		fprintf(stderr, "Could not get a handle to the Polhemus Liberty Device.\n");
		abort();
	}

	if (!liberty_init(handle))
	{
		fprintf(stderr, "Could not initialize the Polhemus Liberty Device. Aborting.\n");
		usb_close(handle);
		return 1;
	}

	buffer_t buf;
	init_buffer(&buf);

	/* activate binary mode */
	liberty_send(handle, "f1\r");

	int n_stations = request_num_of_stations(handle, &buf);
	fprintf(stderr, "found %d stations\n", n_stations);

	/* Valid ammounts of sensors are between 1 and 16 */
	if (n_stations == 0)
	{
		abort();
	}

	/* define which information to get per sensor (called a station
	 by polhemus)

	 o* applies to all stations

	 if this is changed, the station_t struct has to be edited accordingly */
	liberty_send(handle, "o*,8,9,11,3,7\r");
	/* set output hemisphere -- this will produce a response which we're
	 ignoring */
	set_hemisphere(handle, 0, 0, -1);
	/* switch output to centimeters */
	//liberty_send(handle, "u1\r");
	liberty_clear_input(handle); //right now, we just ignore the answer

	station_t *stations = (station_t*) (malloc(sizeof(station_t) * n_stations));
	if (!stations)
		abort();

	/* set up signal handler to catch the interrupt signal */
	signal(SIGINT, signal_handler);

	go_on = 1;

	/* get the time when we begin */
	struct timeval tv;
	gettimeofday(&tv, NULL);
	printf("Timestamp: %d.%06d\n", (unsigned int) (tv.tv_sec), (unsigned int) (tv.tv_usec));

	/* enable continuous mode (get data points continously) */
	liberty_send(handle, "c\r");

	PolhemusROS::PolhemusLiberty polhemus_liberty;
	if (!polhemus_liberty.initialisation(n_stations))
	{
		printf("PolhemusLiberty initialisation failed");
		abort();
	}
	std::vector<KDL::Frame> frames;
	frames.resize(n_stations);
	printf("Put the obstacle at the reference location. Press any key to start calibration.");
	getchar();
	while (ros::ok())
	{
		if (go_on)
		{
			if (!liberty_receive(handle, &buf, stations, sizeof(station_t) * n_stations))
			{
				fprintf(stderr, "receive failed\n");
				return 2;
			}
			/* Note: timestamp is the time in mS after the first read to the
			 system after turning it on
			 at 240Hz, the time between data sample is 1/240 = .00416_
			 seconds.
			 The framecount is a more exact way of finding out the time:
			 timestamp = framecount*1000/240 (rounded down to an int)* */
			for (i = 0; i != n_stations; ++i)
			{
				//printf("%u %u %u %u %f %f %f %f %f %f %f\n", stations[i].timestamp, stations[i].framecount, i, stations[i].distortion, stations[i].x, stations[i].y, stations[i].z, stations[i].quaternion[0], stations[i].quaternion[1], stations[i].quaternion[2], stations[i].quaternion[3]);

				frames[i] =
						KDL::Frame(KDL::Rotation::Quaternion(stations[i].quaternion[0], stations[i].quaternion[1], stations[i].quaternion[2], stations[i].quaternion[3]), KDL::Vector(-stations[i].y
								/ 100, -stations[i].x / 100, -stations[i].z / 100));
			}
			polhemus_liberty.getData(frames);
		}
	}
	/* stop continous mode */
	liberty_send(handle, "p");

	usb_close(handle);

	free(stations);
	stations = NULL;

	return 0;
}

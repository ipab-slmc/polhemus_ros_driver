/*
 * polhemus_ros.h
 *
 *  Created on: 1 Sep 2014
 *      Author: yiming
 */

#ifndef POLHEMUS_ROS_H_
#define POLHEMUS_ROS_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <exotica/MeshVertex.h>
#include <kdl/frames.hpp>
namespace PolhemusROS
{
	//	The Polhemus Liberty is used to transfer polhemus data to ROS topic
	class PolhemusLiberty
	{
		public:
			/**
			 * \brief	Constructor
			 */
			PolhemusLiberty();

			/**
			 * \brief	Destructor
			 */
			~PolhemusLiberty();

			/**
			 * \brief	Initialisation
			 * @param	size	Number of stations
			 * @return	True if succeeded, false otherwise
			 */
			bool initialisation(const int size);

			/**
			 * \brief	Get data from liberty
			 * @param	frames	The data
			 * @return	True if succeeded, false otherwise
			 */
			bool getData(std::vector<KDL::Frame> & frames);

		private:
			//	Base frame ID, -1 if use real base
			int base_id_;

			//	Base offset
			KDL::Frame base_offset_;
 
			//	Number of stations
			int size_;

			//	Initialisation flag
			bool initialised_;

			//	ROS node handle
			ros::NodeHandle nh_;

			//	Pose array
			geometry_msgs::PoseArray poses_;

			//	Data publisher
			ros::Publisher pose_pub_;

			//	Specify goal/obstacle properties
			//	Goal topic
			std::string goal_topic_;

			//	Goal index
			int goal_id_;

			//	Obstacle topic
			std::string obs_topic_;

			//	Obstacle index
			int obs_id_;

			//	Goal publisher
			ros::Publisher goal_pub_;

			//	Obstacle publisher
			ros::Publisher obs_pub_;

			//	Goal pose array
			geometry_msgs::PoseArray goal_poses_;

			//	Obstacle pose
			exotica::MeshVertex obs_poses_;

			bool calibration_;

			KDL::Frame offset_;
	};
}

#endif /* POLHEMUS_ROS_H_ */

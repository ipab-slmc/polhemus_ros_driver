/*
 * polhemus_ros.cpp
 *
 *  Created on: 2 Sep 2014
 *      Author: yiming
 */

#include "polhemus_ros/polhemus_ros.h"

namespace PolhemusROS
{
	PolhemusLiberty::PolhemusLiberty() :
					base_id_(-1),
					size_(0),
					initialised_(false),
					nh_("/PolhemusLiberty"),
					pose_pub_(nh_.advertise<geometry_msgs::PoseArray>("/PolhemusLiberty/Poses", 1)),
					goal_topic_("/dummy_goal/goal"),
					obs_topic_("/dummy_object/external_objects"),
					goal_id_(3),
					obs_id_(1),
					calibration_(true)
	{
		//TODO
	}

	PolhemusLiberty::~PolhemusLiberty()
	{
		//TODO
	}

	bool PolhemusLiberty::initialisation(const int size)
	{
		nh_.getParam("/PolhemusLiberty/BaseID", base_id_);
		base_offset_ = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.363, 0.263, 0.045));
		size_ = size;
		poses_.poses.resize(size_);
		poses_.header.frame_id = "/base";

		goal_poses_.poses.resize(1);
		goal_poses_.header.frame_id = "/base";
		obs_poses_.name = "Obstacle1";
		obs_poses_.type = exotica::MeshVertex::OBSTACLE_TO_ALL;
		obs_poses_.toLinks.clear();
		obs_poses_.radius = 0.1;

		goal_pub_ = nh_.advertise<geometry_msgs::PoseArray>(goal_topic_, 1);
		obs_pub_ = nh_.advertise<exotica::MeshVertex>(obs_topic_, 1);
		if (base_id_ >= size_)
			return false;
		initialised_ = true;
		return true;
	}

	bool PolhemusLiberty::getData(std::vector<KDL::Frame> & frames)
	{
		if (!initialised_)
			return false;
		if (frames.size() != size_)
			return false;
//		First Data is used for calibration. LWR Experiment.
		if (calibration_)
		{
			offset_ = frames[obs_id_].Inverse() * frames[base_id_];
			calibration_ = false;
		}

		for (int i = 0; i < size_; i++)
		{
			poses_.poses[i].position.x = frames[i].p.x();
			poses_.poses[i].position.y = frames[i].p.y();
			poses_.poses[i].position.z = frames[i].p.z();
			frames[i].M.GetQuaternion(poses_.poses[i].orientation.x, poses_.poses[i].orientation.y, poses_.poses[i].orientation.z, poses_.poses[i].orientation.w);
		}

		KDL::Frame obs_in_robot = base_offset_ * (frames[base_id_].Inverse()) * frames[obs_id_]
				* offset_;

		poses_.poses[obs_id_].position.x = obs_in_robot.p.x();
		poses_.poses[obs_id_].position.y = obs_in_robot.p.y();
		poses_.poses[obs_id_].position.z = obs_in_robot.p.z();
		obs_in_robot.M.GetQuaternion(poses_.poses[obs_id_].orientation.x, poses_.poses[obs_id_].orientation.y, poses_.poses[obs_id_].orientation.z, poses_.poses[obs_id_].orientation.w);

		poses_.header.stamp = goal_poses_.header.stamp = ros::Time::now();
		goal_poses_.poses[0] = poses_.poses[goal_id_];
		obs_poses_.position = poses_.poses[obs_id_].position;
		pose_pub_.publish(poses_);
		goal_pub_.publish(goal_poses_);
		obs_pub_.publish(obs_poses_);
		return true;
	}
}

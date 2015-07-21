/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "../include/moveit/ompl_interface/eGraphTraj_storage.h"

const std::string ompl_interface::EGraphTrajStorage::DATABASE_NAME =
		"moveit_eGraphTraj_storage";

const std::string ompl_interface::EGraphTrajStorage::EGRAPH_TRAJ_NAME =
		"egraphTraj_id";
const std::string ompl_interface::EGraphTrajStorage::ROBOT_NAME = "robot_id";

ompl_interface::EGraphTrajStorage::EGraphTrajStorage(const std::string &host,
		const unsigned int port, double wait_seconds) :
		MoveItMessageStorage(host, port, wait_seconds) {
	createCollections();
	ROS_INFO("Connected to MongoDB '%s' on host '%s' port '%u'.",
			DATABASE_NAME.c_str(), db_host_.c_str(), db_port_);
}

void ompl_interface::EGraphTrajStorage::createCollections() {
	egraphTraj_collection_.reset(
			new EGraphTrajCollection::element_type(DATABASE_NAME, "eGraphTrajs",
					db_host_, db_port_, timeout_));
}

void ompl_interface::EGraphTrajStorage::reset() {
	egraphTraj_collection_.reset();
	MoveItMessageStorage::drop(DATABASE_NAME);
	createCollections();
}

void ompl_interface::EGraphTrajStorage::addEGraphTraj(
		const egraphmsg::EGraphTraj &msg, const std::string &name,
		const std::string &robot) {
	bool replace = false;
	if (hasEGraphTraj(name, robot)) {
		removeEGraphTraj(name, robot);
		replace = true;
	}
	mongo_ros::Metadata metadata(EGRAPH_TRAJ_NAME, name, ROBOT_NAME, robot);
	ROS_INFO("inserting");
	egraphTraj_collection_->insert(msg, metadata);
	ROS_INFO("%s egraph '%s'", replace ? "Replaced" : "Added", name.c_str());
}

bool ompl_interface::EGraphTrajStorage::hasEGraphTraj(const std::string &name,
		const std::string &robot) const {
	mongo_ros::Query q(EGRAPH_TRAJ_NAME, name);
	if (!robot.empty())
		q.append(ROBOT_NAME, robot);
	std::vector<EGraphTrajWithMetadata> constr =
			egraphTraj_collection_->pullAllResults(q, true);
	return !constr.empty();
}

void ompl_interface::EGraphTrajStorage::getAllEGraphTrajs(
		std::vector<std::string> &names, const std::string &robot) const {
	names.clear();
	mongo_ros::Query q;
	if (!robot.empty())
		q.append(ROBOT_NAME, robot);
	std::vector<EGraphTrajWithMetadata> constr =
			egraphTraj_collection_->pullAllResults(q, true, EGRAPH_TRAJ_NAME,
					true);
	for (std::size_t i = 0; i < constr.size(); ++i)
		if (constr[i]->metadata.hasField(EGRAPH_TRAJ_NAME.c_str()))
			names.push_back(constr[i]->lookupString(EGRAPH_TRAJ_NAME));
}

bool ompl_interface::EGraphTrajStorage::getEGraphTraj(
		EGraphTrajWithMetadata &msg_m, const std::string &name,
		const std::string &robot) const {
	mongo_ros::Query q(EGRAPH_TRAJ_NAME, name);
	if (!robot.empty())
		q.append(ROBOT_NAME, robot);
	std::vector<EGraphTrajWithMetadata> constr =
			egraphTraj_collection_->pullAllResults(q, false);
	if (constr.empty())
		return false;
	else {
		msg_m = constr.front();
		return true;
	}
}

void ompl_interface::EGraphTrajStorage::removeEGraphTraj(
		const std::string &name, const std::string &robot) {
	mongo_ros::Query q(EGRAPH_TRAJ_NAME, name);
	if (!robot.empty())
		q.append(ROBOT_NAME, robot);
	unsigned int rem = egraphTraj_collection_->removeMessages(q);
	ROS_INFO("Removed %u eGraphTraj messages (named '%s')", rem, name.c_str());
}

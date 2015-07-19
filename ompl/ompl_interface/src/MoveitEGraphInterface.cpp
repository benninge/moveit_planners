#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_

#include <moveit/ompl_interface/MoveitEGraphInterface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/State.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

ompl_interface::MoveitEGraphInterface::MoveitEGraphInterface() :
        nh_("~"), id_(0) {

    //init Storage
    storage_Trajs_ = new ompl_interface::EGraphTrajStorage();
    pc_ = ompl_interface_->getLastPlanningContext();

    if (!pc_ || !pc_->getPlanningScene()) {
        ROS_ERROR(
                "MoveitEGraphInterface: No planning context to sample states for");
        return;
    }
    markerArray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "visualization_marker_array", 1000);
}

ompl_interface::MoveitEGraphInterface::~MoveitEGraphInterface() {

}

void ompl_interface::MoveitEGraphInterface::eGraphToMarkerArray(
        moveit_msgs::DisplayTrajectory display_trajectory,
        robot_model::RobotModelConstPtr &robot_model) {
    for (int i = 0; i < display_trajectory.trajectory.size(); i++) {
        trajToMarkerArray(display_trajectory.trajectory[i], robot_model);
    }
    publishMarkerArray(markerArray_pub_);
}

void ompl_interface::MoveitEGraphInterface::trajToMarkerArray(
        moveit_msgs::RobotTrajectory trajectory,
        robot_model::RobotModelConstPtr &robot_model) {
    moveit::core::RobotStatePtr kinematic_state(
            new robot_state::RobotState(robot_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group =
            robot_model->getJointModelGroup("omnirob_lbr");

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "basic_shapes";
    marker.id = id_++;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    geometry_msgs::Point p;
    std::vector<double> v;

    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); i++) {
        v = trajectory.joint_trajectory.points[i].positions;

        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[0], &v[0]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[1], &v[1]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[2], &v[2]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[3], &v[3]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[4], &v[4]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[5], &v[5]);
        kinematic_state->setJointPositions(
                trajectory.joint_trajectory.joint_names[6], &v[6]);
        const Eigen::Affine3d &end_effector_state =
                kinematic_state->getGlobalLinkTransform("lwr_joint7_frame");
        geometry_msgs::Pose pose = transformPose(end_effector_state);
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = pose.position.z;
        marker.points.push_back(p);

        //draw start_state
        if (i == 0) {
            nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z,
                    0.0f, 1.0f, 0.0f);
        }

        //draw goal_state
        if (i == trajectory.joint_trajectory.points.size() - 1) {
            nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z,
                    0.0f, 0.0f, 1.0f);
        }

    }

    mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::nodeToMarkerArray(double x,
        double y, double z, float red, float green, float blue) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.ns = "basic_shapes";
    marker.id = id_++;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;

    mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::save(
        std::vector<ompl::geometric::EGraphNode*> eGraph,
        const ompl::base::SpaceInformationPtr &si) {

    //traverse egraph and search for goals
    for (size_t i = 0; i < eGraph.size(); i++) {
        //check if goal
        if (eGraph[i]->children.empty() == true) {
            ompl::geometric::EGraphNode *current_node = eGraph[i];
            std::vector<ompl::geometric::EGraphNode*> node_traj;
            while (current_node != NULL) {
                node_traj.push_back(current_node);
                current_node = current_node->parent;
            }
            moveit_msgs::DisplayTrajectory traj = omplNodesToDisplayTraj(
                    node_traj);
            if (storage_Trajs_->hasEGraphTraj("graph", "robot") == true) {
                addTrajToEGraph(traj, "graph", "robot");
            } else {
                addGraphToStorage(traj, "graph", "graph");
            }
        }
    }
    moveit_msgs::DisplayTrajectory storage_trajectory = getGraphFromStorage(
            "graph", "robot");
    robot_model::RobotModelConstPtr robot_model = pc_->getRobotModel();
    eGraphToMarkerArray(storage_trajectory, robot_model);
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::load(
        const ompl::base::SpaceInformationPtr &si) {
    std::vector<ompl::geometric::EGraphNode*> eGraph;
    moveit_msgs::DisplayTrajectory traj = getGraphFromStorage("graph", "robot");
    eGraph = displayTrajToOmplNodes(traj, si);
    return eGraph;
}

moveit_msgs::DisplayTrajectory ompl_interface::MoveitEGraphInterface::omplNodesToDisplayTraj(
        std::vector<ompl::geometric::EGraphNode*> nodes) {
    moveit_msgs::DisplayTrajectory traj_msg;
    robot_state::RobotState rstate(pc_->getRobotModel());
    robot_trajectory::RobotTrajectory traj(pc_->getRobotModel(),
            pc_->getJointModelGroup());
    for (size_t i = 0; i < nodes.size(); i++) {
        pc_->getOMPLStateSpace()->copyToRobotState(rstate, nodes[i]->state);
        traj.addSuffixWayPoint(rstate, 0.0);
    }
    //rstate.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();

    moveit_msgs::DisplayRobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(rstate, state_msg.state);

    traj_msg.model_id = pc_->getRobotModel()->getName();
    traj_msg.trajectory.resize(1);
    traj.getRobotTrajectoryMsg(traj_msg.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(traj.getFirstWayPoint(),
            traj_msg.trajectory_start);
    return traj_msg;
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::displayTrajToOmplNodes(
        moveit_msgs::DisplayTrajectory traj_msg,
        const ompl::base::SpaceInformationPtr &si) {
    robot_state::RobotState rstate(pc_->getRobotModel());
    std::vector<double> v;
    std::vector<ompl::geometric::EGraphNode*> nodes;

    //traverse trajs
    for (size_t i = 0; i < traj_msg.trajectory.size(); i++) {
        //traverse points on traj
        for (size_t j = 0;
                j < traj_msg.trajectory[i].joint_trajectory.points.size();
                j++) {
            v = traj_msg.trajectory[i].joint_trajectory.points[j].positions;
            //traverse joints on point of traj
            for (size_t k = 0;
                    k
                            < traj_msg.trajectory[i].joint_trajectory.joint_names.size();
                    k++) {
                rstate.setJointPositions(
                        traj_msg.trajectory[i].joint_trajectory.joint_names[k],
                        &v[k]);
            }
            ompl::geometric::EGraphNode *node = new ompl::geometric::EGraphNode(
                    si);
            pc_->getOMPLStateSpace()->copyToOMPLState(node->state, rstate);
            ompl::geometric::EGraphNode *old_node;

            //reconstruct parents and neighbors
            if (j == 0) {
                //start of traj, do nothing
            } else {
                //node->neighbors.push_back(old_node);
                old_node->children.push_back(node);
                node->parent = old_node;
            }

            old_node = node;
            nodes.push_back(node);
        }
    }
    return nodes;

}

bool ompl_interface::MoveitEGraphInterface::addGraphToStorage(
        moveit_msgs::DisplayTrajectory display_trajectory, std::string name,
        std::string robot) {
    egraphmsg::EGraphTraj eGraphTraj;
    eGraphTraj.model_id = display_trajectory.model_id;
    eGraphTraj.trajectory_start = display_trajectory.trajectory_start;
    eGraphTraj.trajectory = display_trajectory.trajectory;
    storage_Trajs_->addEGraphTraj(eGraphTraj, name, robot);
    return storage_Trajs_->hasEGraphTraj(name, robot);
}

bool ompl_interface::MoveitEGraphInterface::addTrajToEGraph(
        moveit_msgs::DisplayTrajectory input_trajectory, std::string graph,
        std::string robot) {
    moveit_msgs::DisplayTrajectory traj = getGraphFromStorage(graph, robot);

//combine traj and input_trajectory
    traj.trajectory.insert(traj.trajectory.end(),
            input_trajectory.trajectory.begin(),
            input_trajectory.trajectory.end());

    egraphmsg::EGraphTraj eGraphTraj;
    eGraphTraj.model_id = traj.model_id;
    eGraphTraj.trajectory_start = traj.trajectory_start;
    eGraphTraj.trajectory = traj.trajectory;
    storage_Trajs_->addEGraphTraj(eGraphTraj, graph, robot);
    return storage_Trajs_->hasEGraphTraj(graph, robot);
}

moveit_msgs::DisplayTrajectory ompl_interface::MoveitEGraphInterface::getGraphFromStorage(
        std::string name, std::string robot) {
    mongo_ros::MessageWithMetadata<egraphmsg::EGraphTraj>::ConstPtr eGraphTrajMeta;
    storage_Trajs_->getEGraphTraj(eGraphTrajMeta, name, robot);

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = eGraphTrajMeta->model_id;
    display_trajectory.trajectory_start = eGraphTrajMeta->trajectory_start;
    display_trajectory.trajectory = eGraphTrajMeta->trajectory;
    return display_trajectory;
}

#endif


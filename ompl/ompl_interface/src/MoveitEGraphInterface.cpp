#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_

#include <moveit/ompl_interface/MoveitEGraphInterface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/State.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

ompl_interface::MoveitEGraphInterface::MoveitEGraphInterface() :
        nh_("~") {

    //init Storage
    storage_Trajs_ = new ompl_interface::EGraphTrajStorage();
    pc_ = ompl_interface_->getLastPlanningContext();

    if (!pc_ || !pc_->getPlanningScene()) {
            ROS_ERROR(
                    "MoveitEGraphInterface: No planning context to sample states for");
            return;
        }
}

ompl_interface::MoveitEGraphInterface::~MoveitEGraphInterface() {

}

void ompl_interface::MoveitEGraphInterface::save(
        std::vector<ompl::geometric::eGraphPlanner::EGraphNode*> eGraph,
        const ompl::base::SpaceInformationPtr &si) {

    //traverse egraph and search for goals
    for (size_t i = 0; i < eGraph.size(); i++) {
        //check if goal
        if (eGraph[i]->children.empty() == true) {
            ompl::geometric::eGraphPlanner::EGraphNode *current_node = eGraph[i];
            std::vector<ompl::geometric::eGraphPlanner::EGraphNode*> node_traj;
            while (current_node != NULL) {
                        node_traj.push_back(current_node);
                        current_node = current_node->parent;
                    }
            if (storage_Trajs_->hasEGraphTraj("graph", "robot") == true) {
                addTrajToEGraph(omplNodesToDisplayTraj(node_traj), "graph", "robot");
            } else {
                addGraphToStorage(omplNodesToDisplayTraj(node_traj), "graph", "graph");
            }
        }
    }
}

void ompl_interface::MoveitEGraphInterface::load(
        std::vector<ompl::geometric::eGraphPlanner::EGraphNode*>& eGraph,
        const ompl::base::SpaceInformationPtr &si) {
    moveit_msgs::DisplayTrajectory traj = getGraphFromStorage("graph", "robot");
    displayTrajToOmplNodes(traj, eGraph, si);
}

moveit_msgs::DisplayTrajectory ompl_interface::MoveitEGraphInterface::omplNodesToDisplayTraj(
        std::vector<ompl::geometric::eGraphPlanner::EGraphNode*> nodes) {
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

void ompl_interface::MoveitEGraphInterface::displayTrajToOmplNodes(
        moveit_msgs::DisplayTrajectory traj_msg,
        std::vector<ompl::geometric::eGraphPlanner::EGraphNode*>& nodes,
        const ompl::base::SpaceInformationPtr &si) {
    robot_state::RobotState rstate(pc_->getRobotModel());
    std::vector<double> v;

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
            ompl::geometric::eGraphPlanner::EGraphNode *node =
                    new ompl::geometric::eGraphPlanner::EGraphNode(si);
            pc_->getOMPLStateSpace()->copyToOMPLState(node->state, rstate);
            ompl::geometric::eGraphPlanner::EGraphNode *old_node;

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


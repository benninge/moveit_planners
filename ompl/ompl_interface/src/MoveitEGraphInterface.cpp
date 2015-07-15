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
    storage_egraph_ = new moveit_warehouse::EGraphStorage();
    storage_Trajs_ = new moveit_warehouse::EGraphTrajStorage();

    pub_valid_states_ = nh_.advertise<moveit_msgs::DisplayRobotState>(
            "ompl_planner_valid_states", 5);
    pub_valid_traj_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(
            "ompl_planner_valid_trajectories", 5);

}

ompl_interface::MoveitEGraphInterface::~MoveitEGraphInterface() {

}

void ompl_interface::MoveitEGraphInterface::save(
        std::vector<ompl::geometric::eGraphPlanner::EGraphNode*> eGraph,
        const ompl::base::SpaceInformationPtr &si) {

    for (size_t i = 0; i < eGraph.size(); i++) {
        //si->getStateSpace()->serialize();
    }

}

void ompl_interface::MoveitEGraphInterface::load() {

}

void ompl_interface::MoveitEGraphInterface::copyToDisplayTraj(
        moveit_msgs::DisplayTrajectory& traj_msg,
        std::vector<ompl::base::State*> states) {
    ompl_interface::ModelBasedPlanningContextPtr pc =
            ompl_interface_->getLastPlanningContext();
    if (!pc || !pc->getPlanningScene()) {
        ROS_ERROR(
                "MoveitEGraphInterface: No planning context to sample states for");
        return;
    }

    robot_state::RobotState rstate(pc->getRobotModel());
    robot_trajectory::RobotTrajectory traj(pc->getRobotModel(), pc->getJointModelGroup());
    for (size_t i = 0; i < states.size(); i++) {
        pc->getOMPLStateSpace()->copyToRobotState(rstate, states[i]);
        traj.addSuffixWayPoint(rstate, 0.0);
    }
    //rstate.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();

    moveit_msgs::DisplayRobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(rstate, state_msg.state);
    //pub_valid_states_.publish(state_msg);

    traj_msg.model_id = pc->getRobotModel()->getName();
    traj_msg.trajectory.resize(1);
    traj.getRobotTrajectoryMsg(traj_msg.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(traj.getFirstWayPoint(),
            traj_msg.trajectory_start);
    //pub_valid_traj_.publish(traj_msg);
    if (storage_Trajs_->hasEGraphTraj("debugTrajs", "robot_name") == false) {
        addTrajToEGraph(traj_msg, "debugTrajs", "robot_name");
    } else {
        addGraphToStorage(traj_msg, "debugTrajs", "robot_name");
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

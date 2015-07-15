#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_

#include <ros/ros.h>
#include <ompl/geometric/planners/rrt/eGraphPlanner.h>
#include <moveit/warehouse/moveit_message_storage.h>
#include "../../../../../../egraph/include/eGraph_storage.h"
#include "../../../../../../egraph/include/eGraphTraj_storage.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <limits>
#include <vector>
#include <utility>

//using namespace moveit_warehouse;

namespace ompl_interface {

class MoveitEGraphInterface {
    friend class eGraphPlanner;
public:

    MoveitEGraphInterface();

    virtual ~MoveitEGraphInterface();

    void load();

    void save(std::vector<ompl::geometric::eGraphPlanner::EGraphNode*> eGraph,
            const ompl::base::SpaceInformationPtr &si);

    void copyToDisplayTraj(moveit_msgs::DisplayTrajectory& traj_msg,
            std::vector<ompl::base::State*> states);

protected:
    bool addGraphToStorage(moveit_msgs::DisplayTrajectory display_trajectory,
            std::string name, std::string robot);
    bool addTrajToEGraph(moveit_msgs::DisplayTrajectory input_trajectory,
            std::string graph, std::string robot);
    moveit_msgs::DisplayTrajectory getGraphFromStorage(
            std::string name, std::string robot);

    moveit_warehouse::EGraphStorage* storage_egraph_;
    moveit_warehouse::EGraphTrajStorage* storage_Trajs_;

    boost::scoped_ptr<OMPLInterface> ompl_interface_;
    ros::NodeHandle nh_;
    ros::Publisher pub_valid_states_;
    ros::Publisher pub_valid_traj_;
};

}

#endif

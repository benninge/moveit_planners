#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_

#include <ros/ros.h>
#include <ompl/geometric/planners/rrt/eGraphPlanner.h>
#include <ompl/geometric/planners/rrt/BaseEGraphInterface.h>
#include <moveit/warehouse/moveit_message_storage.h>
#include "eGraphTraj_storage.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <limits>
#include <vector>
#include <utility>

//using namespace moveit_warehouse;

namespace ompl_interface {

class MoveitEGraphInterface: public ompl::geometric::BaseEGraphInterface {
    friend class ompl::geometric::eGraphPlanner;
public:

    MoveitEGraphInterface();

    virtual ~MoveitEGraphInterface();

    virtual std::vector<ompl::geometric::EGraphNode*> load(
            const ompl::base::SpaceInformationPtr &si);

    virtual void save(std::vector<ompl::geometric::EGraphNode*> eGraph,
            const ompl::base::SpaceInformationPtr &si);

private:
    moveit_msgs::DisplayTrajectory omplNodesToDisplayTraj(
            std::vector<ompl::geometric::EGraphNode*> nodes);
    std::vector<ompl::geometric::EGraphNode*> displayTrajToOmplNodes(
            moveit_msgs::DisplayTrajectory traj_msg,
            const ompl::base::SpaceInformationPtr &si);
    bool addGraphToStorage(moveit_msgs::DisplayTrajectory display_trajectory,
            std::string name, std::string robot);
    bool addTrajToEGraph(moveit_msgs::DisplayTrajectory input_trajectory,
            std::string graph, std::string robot);
    moveit_msgs::DisplayTrajectory getGraphFromStorage(std::string name,
            std::string robot);

    void eGraphToMarkerArray(moveit_msgs::DisplayTrajectory display_trajectory,
            robot_model::RobotModelConstPtr &robot_model);
    void trajToMarkerArray(moveit_msgs::RobotTrajectory trajectory,
            robot_model::RobotModelConstPtr &robot_model);
    void nodeToMarkerArray(double x, double y, double z, float red, float green,
            float blue);
    void publishMarkerArray(ros::Publisher markerArray_pub) {
        while (markerArray_pub.getNumSubscribers() < 1) {
            ROS_WARN("Please create a subscriber to the markerArray");
            sleep(1);
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker -> FOUND");
        markerArray_pub.publish(mA_);
    }
    geometry_msgs::Pose transformPose(
            const Eigen::Affine3d end_effector_state) {
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);
        return pose;
    }

    EGraphTrajStorage* storage_Trajs_;

    boost::scoped_ptr<OMPLInterface> ompl_interface_;
    ros::NodeHandle nh_;
    ros::Publisher pub_valid_states_;
    ros::Publisher pub_valid_traj_;
    ompl_interface::ModelBasedPlanningContextPtr pc_;
    visualization_msgs::MarkerArray mA_;
    ros::Publisher markerArray_pub_;
    int id_;
};

}

#endif


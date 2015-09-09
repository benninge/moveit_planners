#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterface_

#include <ros/ros.h>
#include <ompl/geometric/planners/rrt/eGraphPlanner.h>
#include <ompl/geometric/planners/rrt/BaseEGraphInterface.h>
#include <moveit/warehouse/moveit_message_storage.h>
#include "eGraph_storage.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread/mutex.hpp>
#include <limits>
#include <vector>
#include <utility>

namespace ompl_interface
{

class MoveitEGraphInterface : public ompl::geometric::BaseEGraphInterface
{
public:
  static MoveitEGraphInterface* getInstance()
  {
    static MoveitEGraphInterface instance;
    return &instance;
  }

  virtual ~MoveitEGraphInterface();

  /** \brief Load eGraph from database */
  virtual std::vector<ompl::geometric::EGraphNode*> load(const ompl::base::SpaceInformationPtr &si);

  /** \brief Save eGraph to database */
  virtual void save(std::vector<ompl::geometric::EGraphNode*> eGraph, const ompl::base::SpaceInformationPtr &si);

  /** \brief Clear eGraph */
  virtual void resetEGraph()
  {
    //mutex.lock();
    ROS_WARN("storage reset");
    eGraph_storage_->reset();
    //mutex.unlock();
  }

  /** \brief remove all Markers */
  virtual void resetMarkers();

  virtual void setStateSpace(ModelBasedStateSpacePtr ssPtr);

private:
  /** \brief draw current eGraph using markers */
  void draw(std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si);

  MoveitEGraphInterface();
  MoveitEGraphInterface(MoveitEGraphInterface const&);
  void operator=(MoveitEGraphInterface const&);

  /** \brief converts eGraphNodes to RobotStates */
  std::vector<egraphmsg::RobotStateNode> omplNodesToRobotStateNodes(std::vector<ompl::geometric::EGraphNode*> nodes);

  /** \brief converts RobotStates to eGraphNodes */
  std::vector<ompl::geometric::EGraphNode*> robotStateNodesToOmplNodes(
      std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si);

  /** \brief save eGraph in database */
  bool addGraphToStorage(std::vector<egraphmsg::RobotStateNode> input_nodes, std::string name, std::string robot);

  /** \brief load eGraph from database */
  std::vector<egraphmsg::RobotStateNode> getGraphFromStorage(std::string name, std::string robot);


  void drawEdge(egraphmsg::RobotStateNode node1, egraphmsg::RobotStateNode node2, int color_scheme, const ompl::base::SpaceInformationPtr &si);

  void robotNodesToMarkerArray(std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si);
  void robotNodeToMarkerArray(egraphmsg::RobotStateNode node, int color_scheme);
  void nodeToMarkerArray(double x, double y, double z, float red, float green, float blue, float alpha);

  void publishMarkerArray(visualization_msgs::MarkerArray mA)
  {
    while (markerArray_pub_.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to visualization_marker_array");
    }
    ROS_WARN_ONCE("Subscriber found");
    markerArray_pub_.publish(mA);
  }

  geometry_msgs::Pose transformPose(const Eigen::Affine3d end_effector_state)
  {
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(end_effector_state, pose);
    return pose;
  }

  /** \brief pointer to database */
  EGraphStorage* eGraph_storage_;

  ros::NodeHandle nh_;
  visualization_msgs::MarkerArray mA_;
  ros::Publisher markerArray_pub_;

  /** \brief id used to uniquely identify eGraphNodes */
  int id_;

  ModelBasedStateSpacePtr ssPtr_;
  boost::mutex mutex;
  boost::mutex resetMarkers_mutex;
};

}

#endif

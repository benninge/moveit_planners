#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_

#include <moveit/ompl_interface/MoveitEGraphInterface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/State.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <egraphmsg/RobotStateNode.h>

ompl_interface::MoveitEGraphInterface::MoveitEGraphInterface() :
    nh_("~"), id_(0)
{
  //init Storage
  eGraph_storage_ = new ompl_interface::EGraphStorage();
  markerArray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
  resetEGraph();
}

ompl_interface::MoveitEGraphInterface::~MoveitEGraphInterface()
{
  delete eGraph_storage_;
}

void ompl_interface::MoveitEGraphInterface::robotNodesToMarkerArray(std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si)
{

  for (int i = 0; i < robot_nodes.size(); i++)
  {
    if (robot_nodes[i].solution_path)
    {
      if (robot_nodes[i].newly_generated)
      {
        robotNodeToMarkerArray(robot_nodes[i], 2);
      }
      else
      {
        robotNodeToMarkerArray(robot_nodes[i], 3);
      }
    }
    else
    {
      robotNodeToMarkerArray(robot_nodes[i], 1);
    }
    for (int j = 0; j < robot_nodes[i].neighbors.size(); j++)
    {
      if (robot_nodes[i].solution_path && robot_nodes[robot_nodes[i].neighbors[j]].solution_path)
      {
        if (robot_nodes[i].newly_generated || robot_nodes[robot_nodes[i].neighbors[j]].newly_generated)
        {
          drawEdge(robot_nodes[i], robot_nodes[robot_nodes[i].neighbors[j]], 2, si);
        }
        else
        {
          drawEdge(robot_nodes[i], robot_nodes[robot_nodes[i].neighbors[j]], 3, si);
        }
      }
      else
      {
        drawEdge(robot_nodes[i], robot_nodes[robot_nodes[i].neighbors[j]], 1, si);
      }
    }
  }
  publishMarkerArray(mA_);
}

void ompl_interface::MoveitEGraphInterface::setStateSpace(ModelBasedStateSpacePtr ssPtr)
{
  ssPtr_ = ssPtr;
}

void ompl_interface::MoveitEGraphInterface::drawEdge(egraphmsg::RobotStateNode node1, egraphmsg::RobotStateNode node2,
                                                     int color_scheme, const ompl::base::SpaceInformationPtr &si)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "basic_shapes";
  marker.id = id_++;
  //ROS_WARN("marker id: %d", id_);
  marker.lifetime = ros::Duration();
  marker.scale.x = 0.01;
  if (color_scheme == 1)
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
  }
  if (color_scheme == 2)
  {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
  }
  if (color_scheme == 3)
  {
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
  }
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;

  ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si);
  std::vector<egraphmsg::RobotStateNode> robot_nodes;
  robot_nodes.push_back(node1);
  robot_nodes.push_back(node2);
  std::vector<ompl::geometric::EGraphNode*> nodes = robotStateNodesToOmplNodes(robot_nodes, si);
  path->append(nodes[0]->state);
  path->append(nodes[1]->state);
  path->interpolate((unsigned int)path->length()* 25);

  for (size_t i = 0; i < path->getStateCount(); i++) {
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    rstate.setToDefaultValues();
    ssPtr_->copyToRobotState(rstate, path->getState(i));
    const Eigen::Affine3d &end_effector_state = rstate.getGlobalLinkTransform("lbr_7_link");
    geometry_msgs::Pose pose = transformPose(end_effector_state);
    p1.x = pose.position.x;
    p1.y = pose.position.y;
    p1.z = pose.position.z;
    marker.points.push_back(p1);
  }

/*
  robot_state::RobotState rstate1(ssPtr_->getRobotModel());
  rstate1.setToDefaultValues();
  moveit::core::robotStateMsgToRobotState(node1.robotstate, rstate1);

  const Eigen::Affine3d &end_effector_state1 = rstate1.getGlobalLinkTransform("lbr_7_link");
  geometry_msgs::Pose pose1 = transformPose(end_effector_state1);
  p1.x = pose1.position.x;
  p1.y = pose1.position.y;
  p1.z = pose1.position.z;

  robot_state::RobotState rstate2(ssPtr_->getRobotModel());
  rstate2.setToDefaultValues();
  moveit::core::robotStateMsgToRobotState(node2.robotstate, rstate2);
  const Eigen::Affine3d &end_effector_state2 = rstate2.getGlobalLinkTransform("lbr_7_link");
  geometry_msgs::Pose pose2 = transformPose(end_effector_state2);
  p2.x = pose2.position.x;
  p2.y = pose2.position.y;
  p2.z = pose2.position.z;
  marker.points.push_back(p1);
  marker.points.push_back(p2); */
  mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::robotNodeToMarkerArray(egraphmsg::RobotStateNode node, int color_scheme)
{
  robot_state::RobotState rstate(ssPtr_->getRobotModel());
  rstate.setToDefaultValues();
  moveit::core::robotStateMsgToRobotState(node.robotstate, rstate);

  const Eigen::Affine3d &end_effector_state = rstate.getGlobalLinkTransform("lbr_7_link");
  geometry_msgs::Pose pose = transformPose(end_effector_state);
  if (color_scheme == 1)
  {
    nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z, 1.0f, 0.0f, 0.0f, 1.0f);
  }
  if (color_scheme == 2)
  {
    nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z, 0.0f, 1.0f, 0.0f, 1.0f);
  }
  if (color_scheme == 3)
  {
    nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z, 1.0f, 1.0f, 0.0f, 1.0f);
  }

}

void ompl_interface::MoveitEGraphInterface::nodeToMarkerArray(double x, double y, double z, float red, float green,
                                                              float blue, float alpha)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.ns = "basic_shapes";
  marker.id = id_++;
  //ROS_WARN("marker id: %d", id_);
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;

  mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::save(std::vector<ompl::geometric::EGraphNode*> eGraph,
                                                 const ompl::base::SpaceInformationPtr &si)
{
  mutex.lock();
  std::vector<egraphmsg::RobotStateNode> robot_nodes = omplNodesToRobotStateNodes(eGraph);
  //resetEGraph();
  bool b1 = addGraphToStorage(robot_nodes, "graph", "robot");
  ROS_WARN("graph add success?: " + b1 ? "true" : "false");
  resetMarkers();
  draw(robot_nodes, si);
  mutex.unlock();
}

void ompl_interface::MoveitEGraphInterface::draw(std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si)
{
  robotNodesToMarkerArray(robot_nodes, si);
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::load(
    const ompl::base::SpaceInformationPtr &si)
{
  mutex.lock();
  std::vector<ompl::geometric::EGraphNode*> eGraph;
  if (eGraph_storage_->hasEGraph("graph", "robot") == true)
  {
    ROS_INFO("loading");
    std::vector<egraphmsg::RobotStateNode> robot_nodes = getGraphFromStorage("graph", "robot");
    eGraph = robotStateNodesToOmplNodes(robot_nodes, si);
  }
  else
  {
    ROS_INFO("no graph saved in database");
  }
  mutex.unlock();
  return eGraph;

}

void ompl_interface::MoveitEGraphInterface::resetMarkers()
{
  resetMarkers_mutex.lock();
  visualization_msgs::MarkerArray mADelete;
  while (!mA_.markers.empty())
  {
    visualization_msgs::Marker marker = *mA_.markers.begin();
    mA_.markers.erase(mA_.markers.begin());
    marker.action = visualization_msgs::Marker::DELETE;
    mADelete.markers.push_back(marker);
  }
  publishMarkerArray(mADelete);
  resetMarkers_mutex.unlock();
}

std::vector<egraphmsg::RobotStateNode> ompl_interface::MoveitEGraphInterface::omplNodesToRobotStateNodes(
    std::vector<ompl::geometric::EGraphNode*> nodes)
{
  std::vector<egraphmsg::RobotStateNode> robot_nodes;
  //robot_state::RobotState rstate(ssPtr_->getRobotModel());
  int32_t test;
  for (size_t i = 0; i < nodes.size(); i++)
  {
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    rstate.setToDefaultValues();
    ssPtr_->copyToRobotState(rstate, nodes[i]->state);

    egraphmsg::RobotStateNode rstate_node;
    moveit_msgs::RobotState rstatemsg;
    moveit::core::robotStateToRobotStateMsg(rstate, rstatemsg);
    rstate_node.robotstate = rstatemsg;
    rstate_node.solution_path = nodes[i]->solution_path;
    rstate_node.newly_generated = nodes[i]->newly_generated;
    for (size_t j = 0; j < nodes[i]->neighbors.size(); j++)
    {
      if (nodes[i]->neighbors[j]->id == -1)
        ROS_ERROR("node id is -1");

      rstate_node.neighbors.push_back(nodes[i]->neighbors[j]->id);
    }
    robot_nodes.push_back(rstate_node);
  }
  return robot_nodes;
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::robotStateNodesToOmplNodes(
    std::vector<egraphmsg::RobotStateNode> robot_nodes, const ompl::base::SpaceInformationPtr &si)
{

  std::vector<ompl::geometric::EGraphNode*> nodes;
  //traverse robot nodes
  for (size_t i = 0; i < robot_nodes.size(); i++)
  {
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    rstate.setToDefaultValues();

    ompl::geometric::EGraphNode *node = new ompl::geometric::EGraphNode(si);
    moveit::core::robotStateMsgToRobotState(robot_nodes[i].robotstate, rstate);

    //DEBUG
    if (i == 0) {
    const Eigen::Affine3d &end_effector_state = rstate.getGlobalLinkTransform("lbr_7_link");
    geometry_msgs::Pose pose = transformPose(end_effector_state);
    ROS_ERROR("load DEBUG: node %p, pos xyz: %f, %f, %f", node, pose.position.x, pose.position.y, pose.position.z); }

    ssPtr_->copyToOMPLState(node->state, rstate);
    node->id = i;
    node->saved = true;
    nodes.push_back(node);
  }
  for (size_t i = 0; i < nodes.size(); i++)
  {
    for (size_t j = 0; j < robot_nodes[i].neighbors.size(); j++)
    {
      int neighbor = robot_nodes[i].neighbors[j];
      nodes[i]->neighbors.push_back(nodes[neighbor]);
    }
  }

  return nodes;

}

bool ompl_interface::MoveitEGraphInterface::addGraphToStorage(std::vector<egraphmsg::RobotStateNode> input_nodes,
                                                              std::string name, std::string robot)
{
  egraphmsg::EGraph eGraph;
  eGraph.nodes = input_nodes;
  eGraph_storage_->addEGraph(eGraph, name, robot);
  return eGraph_storage_->hasEGraph(name, robot);
}

std::vector<egraphmsg::RobotStateNode> ompl_interface::MoveitEGraphInterface::getGraphFromStorage(std::string name,
                                                                                                  std::string robot)
{
  //ROS_WARN("getGraphFromStorage");
  mongo_ros::MessageWithMetadata<egraphmsg::EGraph>::ConstPtr eGraphMeta;
  eGraph_storage_->getEGraph(eGraphMeta, name, robot);
  return eGraphMeta->nodes;

}

#endif


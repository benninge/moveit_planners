#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MoveitEGraphInterfaceCPP_

#include <moveit/ompl_interface/MoveitEGraphInterface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/State.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <egraphmsg/RobotStateNode.h>

ompl_interface::MoveitEGraphInterface::MoveitEGraphInterface(
        ModelBasedStateSpacePtr ssPtr) :
        nh_("~"), id_(0) {

    //init Storage
    eGraph_storage_ = new ompl_interface::EGraphStorage();
    ssPtr_ = ssPtr;
    markerArray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "visualization_marker_array", 5);
    resetEGraph();
}

ompl_interface::MoveitEGraphInterface::~MoveitEGraphInterface() {

}

void ompl_interface::MoveitEGraphInterface::robotNodesToMarkerArray(
        std::vector<egraphmsg::RobotStateNode> robot_nodes,
        robot_model::RobotModelConstPtr &robot_model, int color_scheme) {

    for (int i = 0; i < robot_nodes.size(); i++) {
        robotNodeToMarkerArray(robot_nodes[i], robot_model, color_scheme);
        for (int j = 0; j < robot_nodes[i].neighbors.size(); j++) {
            drawEdge(robot_nodes[i], robot_nodes[robot_nodes[i].neighbors[j]]);
        }
    }
    publishMarkerArray(mA_);
}
void ompl_interface::MoveitEGraphInterface::drawEdge(egraphmsg::RobotStateNode node1, egraphmsg::RobotStateNode node2) {
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
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    robot_state::RobotState rstate1(ssPtr_->getRobotModel());
    moveit::core::robotStateMsgToRobotState(node1.robotstate, rstate1);

    const Eigen::Affine3d &end_effector_state1 =
                    rstate1.getGlobalLinkTransform("lwr_joint7_frame");
    geometry_msgs::Pose pose1 = transformPose(end_effector_state1);
    p1.x = pose1.position.x;
    p1.y = pose1.position.y;
    p1.z = pose1.position.z;
    robot_state::RobotState rstate2(ssPtr_->getRobotModel());
    moveit::core::robotStateMsgToRobotState(node2.robotstate, rstate2);
    const Eigen::Affine3d &end_effector_state2 =
                    rstate2.getGlobalLinkTransform("lwr_joint7_frame");
    geometry_msgs::Pose pose2 = transformPose(end_effector_state2);
    p2.x = pose2.position.x;
    p2.y = pose2.position.y;
    p2.z = pose2.position.z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::robotNodeToMarkerArray(
        egraphmsg::RobotStateNode node,
        robot_model::RobotModelConstPtr &robot_model, int color_scheme) {
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    moveit::core::robotStateMsgToRobotState(node.robotstate, rstate);

    //moveit::core::RobotStatePtr kinematic_state(rstate);


    const Eigen::Affine3d &end_effector_state =
                    rstate.getGlobalLinkTransform("lwr_joint7_frame");
    geometry_msgs::Pose pose = transformPose(end_effector_state);
    nodeToMarkerArray(pose.position.x, pose.position.y, pose.position.z,
                        0.0f, 0.0f, 1.0f, 0.5f);
}

void ompl_interface::MoveitEGraphInterface::nodeToMarkerArray(double x,
        double y, double z, float red, float green, float blue, float alpha) {
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
    marker.color.a = alpha;

    mA_.markers.push_back(marker);
}

void ompl_interface::MoveitEGraphInterface::save(
        std::vector<ompl::geometric::EGraphNode*> eGraph,
        const ompl::base::SpaceInformationPtr &si) {
    mutex.lock();

    std::vector<egraphmsg::RobotStateNode> robot_nodes =
            omplNodesToRobotStateNodes(eGraph);

    bool b1 = addGraphToStorage(robot_nodes, "graph", "robot");
    ROS_WARN("graph add success?: " + b1 ? "true" : "false");
    draw();
    mutex.unlock();
}

void ompl_interface::MoveitEGraphInterface::draw() {
    if (eGraph_storage_->hasEGraph("graph", "robot") == true) {
        std::vector<egraphmsg::RobotStateNode> robot_nodes =
                getGraphFromStorage("graph", "robot");
        robot_model::RobotModelConstPtr robot_model = ssPtr_->getRobotModel();
        resetMarkers();
        robotNodesToMarkerArray(robot_nodes, robot_model, 1);
    }
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::load(
        const ompl::base::SpaceInformationPtr &si) {
    mutex.lock();
    std::vector<ompl::geometric::EGraphNode*> eGraph;
    if (eGraph_storage_->hasEGraph("graph", "robot") == true) {
        ROS_INFO("loading");
        std::vector<egraphmsg::RobotStateNode> robot_nodes =
                getGraphFromStorage("graph", "robot");
        eGraph = robotStateNodesToOmplNodes(robot_nodes, si);
    } else {
        ROS_INFO("no graph saved in database");
    }
    mutex.unlock();
    return eGraph;

}

void ompl_interface::MoveitEGraphInterface::resetMarkers() {
    resetMarkers_mutex.lock();
    visualization_msgs::MarkerArray mADelete;
    while (!mA_.markers.empty()) {
        visualization_msgs::Marker marker = *mA_.markers.begin();
        mA_.markers.erase(mA_.markers.begin());
        marker.action = visualization_msgs::Marker::DELETE;
        mADelete.markers.push_back(marker);
    }
    publishMarkerArray(mADelete);
    resetMarkers_mutex.unlock();
}

std::vector<egraphmsg::RobotStateNode> ompl_interface::MoveitEGraphInterface::omplNodesToRobotStateNodes(
        std::vector<ompl::geometric::EGraphNode*> nodes) {
    std::vector<egraphmsg::RobotStateNode> robot_nodes;
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    int32_t test;
    for (size_t i = 0; i < nodes.size(); i++) {
        ssPtr_->copyToRobotState(rstate, nodes[i]->state);
        egraphmsg::RobotStateNode rstate_node;
        moveit_msgs::RobotState rstatemsg;
        moveit::core::robotStateToRobotStateMsg(rstate, rstatemsg);
        rstate_node.robotstate = rstatemsg;
        for (size_t j = 0; j < nodes[i]->neighbors.size(); j++) {
            if (nodes[i]->neighbors[j]->id == -1)
                ROS_ERROR("node id is -1");

            rstate_node.neighbors.push_back(nodes[i]->neighbors[j]->id);
        }
        robot_nodes.push_back(rstate_node);
    }
    return robot_nodes;
}

std::vector<ompl::geometric::EGraphNode*> ompl_interface::MoveitEGraphInterface::robotStateNodesToOmplNodes(
        std::vector<egraphmsg::RobotStateNode> robot_nodes,
        const ompl::base::SpaceInformationPtr &si) {

    std::vector<ompl::geometric::EGraphNode*> nodes;
    robot_state::RobotState rstate(ssPtr_->getRobotModel());
    //traverse robot nodes
    for (size_t i = 0; i < robot_nodes.size(); i++) {
        ompl::geometric::EGraphNode *node = new ompl::geometric::EGraphNode(si);
        moveit::core::robotStateMsgToRobotState(robot_nodes[i].robotstate,
                rstate);
        ssPtr_->copyToOMPLState(node->state, rstate);
        node->id = i;
        node->saved = true;
        nodes.push_back(node);
    }
    for (size_t i = 0; i < nodes.size(); i++) {
        for (size_t j = 0; j < robot_nodes[i].neighbors.size(); j++) {
            int neighbor = robot_nodes[i].neighbors[j];
            nodes[i]->neighbors.push_back(nodes[neighbor]);
        }
    }

    return nodes;

}

bool ompl_interface::MoveitEGraphInterface::addGraphToStorage(
        std::vector<egraphmsg::RobotStateNode> input_nodes, std::string name,
        std::string robot) {
    egraphmsg::EGraph eGraph;
        eGraph.nodes = input_nodes;
    eGraph_storage_->addEGraph(eGraph, name, robot);
    return eGraph_storage_->hasEGraph(name, robot);
}

std::vector<egraphmsg::RobotStateNode> ompl_interface::MoveitEGraphInterface::getGraphFromStorage(
        std::string name, std::string robot) {
    //ROS_WARN("getGraphFromStorage");
    mongo_ros::MessageWithMetadata<egraphmsg::EGraph>::ConstPtr eGraphMeta;
    eGraph_storage_->getEGraph(eGraphMeta, name, robot);
    return eGraphMeta->nodes;

}

#endif


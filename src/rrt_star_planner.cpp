/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner {

RRTStarPlanner::RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap_ros);
}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2D* costmap,
                               std::string global_frame) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap, global_frame);
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
  if (!initialized_) {
    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.5);
    private_nh.param("radius", radius_, 1.0);
    private_nh.param("epsilon", epsilon_, 0.2);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_num_nodes", min_num_nodes_, 500);
    private_nh.param("method", method_, 0); // 0: rrt_star, 1: rrt

    graph_pub = private_nh.advertise<visualization_msgs::Marker>("rrt_graph_marker", 0);
    path_pub = private_nh.advertise<nav_msgs::Path>("rrt_path", 0);
    elips_pub = private_nh.advertise<visualization_msgs::Marker>("elips_marker", 0);

    // TODO(Rafael) remove hard coding
    if (search_specific_area_) {
      map_width_ = 10.0;
      map_height_ = 10.0;
    } else {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    ROS_INFO("RRT* Global Planner initialized successfully.");
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {
  // clear the plan, just in case
  plan.clear();

  ROS_INFO("RRT* Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

  std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
  std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};

  planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                  goal_point,
                                                  costmap_,
                                                  goal_tolerance_,
                                                  radius_,
                                                  epsilon_,
                                                  max_num_nodes_,
                                                  min_num_nodes_,
                                                  map_width_,
                                                  map_height_,
                                                  method_));

  std::list<std::pair<float, float>> path;

  if (planner_->pathPlanning(path)) {
    ROS_INFO("RRT* Global Planner: Path found!!!!");
    computeFinalPlan(plan, path);


    if(method_ == 2) {
      std::vector<float> elips = planner_->getElips();
      visualization_msgs::Marker marker_elips;
      marker_elips.header.frame_id = "map";
      marker_elips.header.stamp = ros::Time();
      marker_elips.ns = "elips";
      marker_elips.id = 0;
      marker_elips.type = visualization_msgs::Marker::CYLINDER;
      marker_elips.action = visualization_msgs::Marker::ADD;

      marker_elips.pose.position.x = elips.at(0);
      marker_elips.pose.position.y = elips.at(1);
      marker_elips.pose.position.z = 0.2;
      
      tf2::Quaternion q;
      q.setRPY(0,0,elips.at(2));
      q=q.normalize();

      marker_elips.pose.orientation.x = q[0];
      marker_elips.pose.orientation.y = q[1];
      marker_elips.pose.orientation.z = q[2];
      marker_elips.pose.orientation.w = q[3];

      marker_elips.scale.x = elips.at(3);
      marker_elips.scale.y = elips.at(4);
      marker_elips.scale.z = 0.1;
      marker_elips.color.a = 0.5; // Don't forget to set the alpha!
      marker_elips.color.r = 1.0;
      marker_elips.color.g = 0.0;
      marker_elips.color.b = 1.0;

      elips_pub.publish(marker_elips);

    }

    std::vector<Node> nodes = planner_->getNodes();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "graph";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    geometry_msgs::Point p;
    
    for(int i=0;i<nodes.size();i++)
    {
      int parent_id = nodes.at(i).parent_id;

      if(parent_id != -1)
      {
        p.x = nodes.at(i).x;
        p.y = nodes.at(i).y;
        p.z = 0.2;  
        marker.points.push_back(p);
      
        p.x = nodes.at(parent_id).x;
        p.y = nodes.at(parent_id).y;
        p.z = 0.2;    
        marker.points.push_back(p);
      }      
    }
    graph_pub.publish(marker);

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time();

    std::list<std::pair<float, float>> rrt_path = planner_->getGlobalPath();
    path.poses.clear();
    std::list<std::pair<float, float>>::iterator it;
    for (it = rrt_path.begin(); it != rrt_path.end(); it++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = it->first;     
        pose.pose.position.y = it->second;
        pose.pose.position.z = 0.3;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }
    path_pub.publish(path);

    return true;
  } else {
    ROS_WARN("The planner failed to find a path, choose other goal positions");
    return false;
  }
}

void  RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path) {
  // clean plan
  plan.clear();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
}

}  // namespace rrt_star_global_planner

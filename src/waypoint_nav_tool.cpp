#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include <functional>
#include <sstream>
#include <string>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <rviz_rendering/mesh_loader.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include "waypoint_nav_tool.h"

namespace waypoint_nav_plugin
{

WaypointNavTool::WaypointNavTool()
: moving_flag_node_(nullptr)
, frame_dock_(nullptr)
, frame_(nullptr)
, unique_ind_(0)
{
  shortcut_key_ = 'l';
}

WaypointNavTool::~WaypointNavTool()
{
  for (auto& kv : sn_map_)
    scene_manager_->destroySceneNode(kv.second);

  delete frame_;
  // frame_dock_ is owned by the dock manager; do not delete it
}

void WaypointNavTool::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  flag_resource_ = "package://waypoint_navigation_plugin/media/flag.dae";
  if (!rviz_rendering::loadMeshFromResource(flag_resource_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WaypointNavTool: failed to load model resource '%s'.",
                 flag_resource_.c_str());
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);

  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "waypoint_nav", node_);

  rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new WaypointFrame(context_, &sn_map_, server_.get(), &unique_ind_, nullptr, this);

  if (window_context)
    frame_dock_ = window_context->addPane("Waypoint Navigation", frame_);

  frame_->enable();

  menu_handler_.insert("Delete",
    std::bind(&WaypointNavTool::processFeedback, this, std::placeholders::_1));
  menu_handler_.insert("Set Manual",
    std::bind(&WaypointNavTool::processFeedback, this, std::placeholders::_1));
}

void WaypointNavTool::activate()
{
  if (moving_flag_node_)
    moving_flag_node_->setVisible(true);
}

void WaypointNavTool::deactivate()
{
  if (moving_flag_node_)
    moving_flag_node_->setVisible(false);
}

int WaypointNavTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  if (!moving_flag_node_)
    return Render;

  double height = frame_->getDefaultHeight();

  rviz_rendering::ViewportProjectionFinder finder;
  auto [hit, intersection] = finder.getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);

  if (hit) {
    intersection.z = static_cast<float>(height);
    moving_flag_node_->setVisible(true);
    moving_flag_node_->setPosition(intersection);
    frame_->setWpLabel(intersection);

    for (auto sn_it = sn_map_.begin(); sn_it != sn_map_.end(); ++sn_it) {
      Ogre::Vector3 stored_pos = sn_it->second->getPosition();
      double distance = std::sqrt(
          std::pow(stored_pos.x - intersection.x, 2) +
          std::pow(stored_pos.y - intersection.y, 2));

      if (distance < 0.4) {
        moving_flag_node_->setVisible(false);
        if (event.rightDown()) {
          sn_it->second->detachAllObjects();
          std::stringstream wp_name;
          wp_name << g_wp_name_prefix << sn_it->first;
          server_->erase(wp_name.str());
          server_->applyChanges();
          sn_map_.erase(sn_it);
          moving_flag_node_->setVisible(true);
          return Render | Finished;
        }
      }
    }

    if (event.leftDown()) {
      Ogre::Quaternion quat;
      makeIm(intersection, quat, frame_->ui_->sixDcheckBox->checkState() == Qt::Checked);
      return Render | Finished;
    }
  } else {
    moving_flag_node_->setVisible(false);
  }
  return Render;
}

void WaypointNavTool::makeIm(const Ogre::Vector3& position,
                              const Ogre::Quaternion& quat, bool full_dof)
{
  unique_ind_++;

  std::stringstream wp_name;
  wp_name << g_wp_name_prefix << unique_ind_;
  std::string wp_name_str(wp_name.str());

  if (!rviz_rendering::loadMeshFromResource(flag_resource_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WaypointNavTool: failed to load model resource '%s'.",
                 flag_resource_.c_str());
    return;
  }

  Ogre::SceneNode* sn_ptr = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  sn_ptr->attachObject(entity);
  sn_ptr->setVisible(true);
  sn_ptr->setPosition(position);
  sn_ptr->setOrientation(quat);

  if (sn_map_.find(unique_ind_) != sn_map_.end()) {
    RCLCPP_WARN(node_->get_logger(), "%s already in map", wp_name_str.c_str());
    return;
  }
  sn_map_.insert(std::make_pair(unique_ind_, sn_ptr));

  frame_->setWpCount(static_cast<int>(sn_map_.size()));

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.stamp = node_->now();
  int_marker.header.frame_id = frame_->getFrameId().toStdString();
  int_marker.pose.position.x = position.x;
  int_marker.pose.position.y = position.y;
  int_marker.pose.position.z = position.z;
  int_marker.pose.orientation.x = quat.x;
  int_marker.pose.orientation.y = quat.y;
  int_marker.pose.orientation.z = quat.z;
  int_marker.pose.orientation.w = quat.w;
  int_marker.scale = 2;
  int_marker.name = wp_name_str;

  visualization_msgs::msg::Marker cyn_marker;
  cyn_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  cyn_marker.scale.x = 2.0;
  cyn_marker.scale.y = 2.0;
  cyn_marker.scale.z = 0.2;
  cyn_marker.color.r = 0.5f;
  cyn_marker.color.g = 0.5f;
  cyn_marker.color.b = 0.5f;
  cyn_marker.color.a = 0.5f;

  visualization_msgs::msg::InteractiveMarkerControl cyn_control;
  cyn_control.always_visible = true;
  cyn_control.markers.push_back(cyn_marker);
  int_marker.controls.push_back(cyn_control);

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 0.707106781;
  control.orientation.x = 0;
  control.orientation.y = 0.707106781;
  control.orientation.z = 0;
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  if (full_dof) {
    control.orientation.w = 0.707106781;
    control.orientation.x = 0.707106781;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 0.707106781;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0.707106781;
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_delete";
  control.description = wp_name_str;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name,
    std::bind(&WaypointNavTool::processFeedback, this, std::placeholders::_1));
  menu_handler_.apply(*server_, int_marker.name);

  Ogre::Vector3 p = position;
  Ogre::Quaternion q = quat;
  frame_->setSelectedMarkerName(wp_name_str);
  frame_->setWpLabel(p);
  frame_->setPose(p, q);

  server_->applyChanges();
}

void WaypointNavTool::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;

  switch (feedback->event_type) {
    case Feedback::MENU_SELECT: {
      auto sn_entry = sn_map_.find(
          std::stoi(feedback->marker_name.substr(strlen(g_wp_name_prefix))));
      if (sn_entry == sn_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                     feedback->marker_name.c_str());
      } else if (feedback->menu_entry_id == 1) {
        std::stringstream wp_name;
        wp_name << g_wp_name_prefix << sn_entry->first;
        server_->erase(wp_name.str());
        menu_handler_.reApply(*server_);
        server_->applyChanges();
        sn_entry->second->detachAllObjects();
        sn_map_.erase(sn_entry);
        frame_->setWpCount(static_cast<int>(sn_map_.size()));
      } else {
        Ogre::Vector3 position;
        Ogre::Quaternion quat;
        frame_->getPose(position, quat);

        geometry_msgs::msg::Pose pos;
        pos.position.x = position.x;
        pos.position.y = position.y;
        pos.position.z = position.z;
        pos.orientation.x = quat.x;
        pos.orientation.y = quat.y;
        pos.orientation.z = quat.z;
        pos.orientation.w = quat.w;

        sn_entry->second->setPosition(position);
        sn_entry->second->setOrientation(quat);
        frame_->setWpLabel(position);
        server_->setPose(feedback->marker_name, pos);
        server_->applyChanges();
      }
      break;
    }
    case Feedback::POSE_UPDATE: {
      auto sn_entry = sn_map_.find(
          std::stoi(feedback->marker_name.substr(strlen(g_wp_name_prefix))));
      if (sn_entry == sn_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                     feedback->marker_name.c_str());
      } else {
        Ogre::Vector3 position;
        position.x = feedback->pose.position.x;
        position.y = feedback->pose.position.y;
        position.z = feedback->pose.position.z;

        Ogre::Quaternion quat;
        quat.x = feedback->pose.orientation.x;
        quat.y = feedback->pose.orientation.y;
        quat.z = feedback->pose.orientation.z;
        quat.w = feedback->pose.orientation.w;

        sn_entry->second->setPosition(position);
        sn_entry->second->setOrientation(quat);
        frame_->setWpLabel(position);
        frame_->setPose(position, quat);
        frame_->setSelectedMarkerName(feedback->marker_name);
      }
      break;
    }
  }
}

void WaypointNavTool::getMarkerPoses()
{
  for (auto& kv : sn_map_) {
    std::stringstream wp_name;
    wp_name << g_wp_name_prefix << kv.first;
    visualization_msgs::msg::InteractiveMarker int_marker;
    server_->get(wp_name.str(), int_marker);
    RCLCPP_INFO(node_->get_logger(), "pos: %g %g %g",
                int_marker.pose.position.x,
                int_marker.pose.position.y,
                int_marker.pose.position.z);
  }
}

void WaypointNavTool::clearAllWaypoints()
{
  for (auto& kv : sn_map_)
    scene_manager_->destroySceneNode(kv.second);
  sn_map_.clear();
  unique_ind_ = 0;
}

void WaypointNavTool::save(rviz_common::Config config) const
{
  config.mapSetValue("Class", getClassId());
  rviz_common::Config waypoints_config = config.mapMakeChild("WaypointsTool");
  waypoints_config.mapSetValue("topic", frame_->getOutputTopic());
  waypoints_config.mapSetValue("frame_id", frame_->getFrameId());
  waypoints_config.mapSetValue("default_height", frame_->getDefaultHeight());
}

void WaypointNavTool::load(const rviz_common::Config& config)
{
  rviz_common::Config waypoints_config = config.mapGetChild("WaypointsTool");
  QString topic, frame;
  float height = 0.0f;
  if (!waypoints_config.mapGetString("topic", &topic))
    topic = "/waypoints";
  if (!waypoints_config.mapGetString("frame_id", &frame))
    frame = "map";
  waypoints_config.mapGetFloat("default_height", &height);
  frame_->setConfig(topic, frame, height);
}

}  // namespace waypoint_nav_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavTool, rviz_common::Tool)

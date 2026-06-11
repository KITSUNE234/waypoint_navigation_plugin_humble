#pragma once

#include <memory>
#include <map>
#include <string>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rviz_common/tool.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

#include "waypoint_nav_frame.h"

#include <OgrePrerequisites.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class PanelDockWidget;
}

namespace waypoint_nav_plugin
{

class WaypointNavTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  WaypointNavTool();
  ~WaypointNavTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

  void makeIm(const Ogre::Vector3& position, const Ogre::Quaternion& quat,
              bool full_dof = false);

private:
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void getMarkerPoses();
  void clearAllWaypoints();

  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;

  WaypointFrame* frame_;
  rviz_common::PanelDockWidget* frame_dock_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  typedef std::map<int, Ogre::SceneNode*> M_StringToSNPtr;
  M_StringToSNPtr sn_map_;

  int unique_ind_;
};

}  // namespace waypoint_nav_plugin

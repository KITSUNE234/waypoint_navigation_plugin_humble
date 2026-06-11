#pragma once

#include <mutex>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "ui_WaypointNavigation.h"

#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>

namespace Ogre
{
class SceneNode;
class SceneManager;
}

namespace rviz_common
{
class DisplayContext;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace Ui
{
class WaypointNavigationWidget;
}

namespace waypoint_nav_plugin
{
class WaypointNavTool;
}

namespace waypoint_nav_plugin
{

constexpr char g_wp_name_prefix[] = "waypoint_";

class WaypointFrame : public QWidget
{
  friend class WaypointNavTool;
  Q_OBJECT

public:
  WaypointFrame(rviz_common::DisplayContext* context,
                std::map<int, Ogre::SceneNode*>* map_ptr,
                interactive_markers::InteractiveMarkerServer* server,
                int* unique_ind,
                QWidget* parent = nullptr,
                WaypointNavTool* wp_tool = nullptr);
  ~WaypointFrame();

  void enable();
  void disable();

  void setWpCount(int size);
  void setConfig(QString topic, QString frame, float height);
  void setWpLabel(Ogre::Vector3 position);
  void setSelectedMarkerName(std::string name);
  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quat);

  double getDefaultHeight();
  QString getFrameId();
  QString getOutputTopic();
  void getPose(Ogre::Vector3& position, Ogre::Quaternion& quat);

  void loadFromYaml(const std::string& filename);
  void loadFromJson(const std::string& filename);
  void saveToYaml(const std::string& filename);
  void saveToJson(const std::string& filename);

protected:
  Ui::WaypointNavigationWidget* ui_;
  rviz_common::DisplayContext* context_;

private Q_SLOTS:
  void publishButtonClicked();
  void clearAllWaypoints();
  void heightChanged(double h);
  void frameChanged();
  void topicChanged();
  void poseChanged(double val);
  void saveButtonClicked();
  void loadButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wp_pub_;

  WaypointNavTool* wp_nav_tool_;
  std::map<int, Ogre::SceneNode*>* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;

  interactive_markers::InteractiveMarkerServer* server_;

  double default_height_;
  QString output_topic_;
  QString frame_id_;

  std::mutex frame_updates_mutex_;
  std::string selected_marker_name_;
};

}  // namespace waypoint_nav_plugin

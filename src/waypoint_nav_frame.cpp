#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include <QFileDialog>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "waypoint_nav_tool.h"

namespace waypoint_nav_plugin
{

struct MissionKeywords {
  inline static const std::string kPosition = "position";
};

WaypointFrame::WaypointFrame(rviz_common::DisplayContext* context,
                             std::map<int, Ogre::SceneNode*>* map_ptr,
                             interactive_markers::InteractiveMarkerServer* server,
                             int* unique_ind,
                             QWidget* parent,
                             WaypointNavTool* wp_tool)
: QWidget(parent)
, context_(context)
, ui_(new Ui::WaypointNavigationWidget())
, sn_map_ptr_(map_ptr)
, unique_ind_(unique_ind)
, server_(server)
, frame_id_("map")
, default_height_(0.0)
, selected_marker_name_(std::string(g_wp_name_prefix) + "1")
, wp_nav_tool_(wp_tool)
{
  scene_manager_ = context_->getSceneManager();
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  ui_->setupUi(this);

  output_topic_ = "waypoints";
  wp_pub_ = node_->create_publisher<nav_msgs::msg::Path>(output_topic_.toStdString(), 1);

  connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
  connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
  connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));
  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));
}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = nullptr;
}

void WaypointFrame::enable()
{
  show();
}

void WaypointFrame::disable()
{
  wp_pub_.reset();
  hide();
}

void WaypointFrame::saveButtonClicked()
{
  QString filename = QFileDialog::getSaveFileName(
      nullptr, tr("Save Mission"), "waypoints",
      tr("Mission Files (*.yaml *.json)"));

  if (filename.isEmpty()) {
    RCLCPP_ERROR(node_->get_logger(), "No mission filename selected");
    return;
  }

  const std::string filename_str = filename.toStdString();
  RCLCPP_INFO(node_->get_logger(), "saving waypoints to %s", filename_str.c_str());

  if (filename.endsWith(".yaml")) {
    saveToYaml(filename_str);
  } else if (filename.endsWith(".json")) {
    saveToJson(filename_str);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Invalid mission file format: %s", filename_str.c_str());
  }
}

void WaypointFrame::saveToYaml(const std::string& filename)
{
  YAML::Emitter out;
  out << YAML::BeginSeq;
  for (auto sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    const Ogre::Vector3 position = sn_it->second->getPosition();
    out << YAML::BeginMap;
    out << YAML::Key << MissionKeywords::kPosition;
    out << YAML::Value << YAML::Flow << YAML::BeginSeq
        << position.x << position.y << position.z
        << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  std::ofstream fout(filename);
  fout << out.c_str();
}

void WaypointFrame::saveToJson(const std::string&) {}

void WaypointFrame::loadButtonClicked()
{
  const QString filename = QFileDialog::getOpenFileName(
      nullptr, tr("Load Mission"), "~/",
      tr("Mission Files (*.yaml *.json)"));

  if (filename.isEmpty()) {
    RCLCPP_ERROR(node_->get_logger(), "No mission file selected");
    return;
  }

  const std::string filename_str = filename.toStdString();
  RCLCPP_INFO(node_->get_logger(), "loading waypoints from %s", filename_str.c_str());

  if (filename.endsWith(".yaml")) {
    loadFromYaml(filename_str);
  } else if (filename.endsWith(".json")) {
    loadFromJson(filename_str);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Invalid mission file format: %s", filename_str.c_str());
  }
}

void WaypointFrame::loadFromYaml(const std::string& filename)
{
  YAML::Node root_node = YAML::LoadFile(filename);
  for (auto it_wpt = root_node.begin(); it_wpt != root_node.end(); ++it_wpt) {
    YAML::Node wpt_node = *it_wpt;
    for (auto it_map = wpt_node.begin(); it_map != wpt_node.end(); ++it_map) {
      const std::string key = it_map->first.as<std::string>();
      if (key == MissionKeywords::kPosition) {
        YAML::Node pos_node = it_map->second;
        Ogre::Vector3 position;
        position.x = pos_node[0].as<double>();
        position.y = pos_node[1].as<double>();
        position.z = pos_node[2].as<double>();
        RCLCPP_WARN(node_->get_logger(),
                    "add waypoint at x=%g y=%g z=%g",
                    position.x, position.y, position.z);
        Ogre::Quaternion quat;
        wp_nav_tool_->makeIm(position, quat,
                             ui_->sixDcheckBox->checkState() == Qt::Checked);
      }
    }
  }
  publishButtonClicked();
}

void WaypointFrame::loadFromJson(const std::string&) {}

void WaypointFrame::publishButtonClicked()
{
  nav_msgs::msg::Path path;
  for (auto sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    Ogre::Vector3 position = sn_it->second->getPosition();
    Ogre::Quaternion quat = sn_it->second->getOrientation();

    geometry_msgs::msg::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;
    path.poses.push_back(pos);
  }
  path.header.frame_id = frame_id_.toStdString();
  if (wp_pub_)
    wp_pub_->publish(path);
}

void WaypointFrame::clearAllWaypoints()
{
  for (auto sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it)
    scene_manager_->destroySceneNode(sn_it->second);

  sn_map_ptr_->clear();
  *unique_ind_ = 0;

  server_->clear();
  server_->applyChanges();
}

void WaypointFrame::heightChanged(double h)
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  default_height_ = h;
}

void WaypointFrame::setSelectedMarkerName(std::string name)
{
  selected_marker_name_ = name;
}

void WaypointFrame::poseChanged(double)
{
  auto sn_entry = sn_map_ptr_->end();
  try {
    const int idx = std::stoi(selected_marker_name_.substr(strlen(g_wp_name_prefix)));
    sn_entry = sn_map_ptr_->find(idx);
  } catch (const std::logic_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    return;
  }

  if (sn_entry == sn_map_ptr_->end()) {
    RCLCPP_ERROR(node_->get_logger(), "%s not found in map",
                 selected_marker_name_.c_str());
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion quat;
  getPose(position, quat);

  sn_entry->second->setPosition(position);
  sn_entry->second->setOrientation(quat);

  std::stringstream wp_name;
  wp_name << g_wp_name_prefix << sn_entry->first;

  visualization_msgs::msg::InteractiveMarker int_marker;
  if (server_->get(wp_name.str(), int_marker)) {
    int_marker.pose.position.x = position.x;
    int_marker.pose.position.y = position.y;
    int_marker.pose.position.z = position.z;
    int_marker.pose.orientation.x = quat.x;
    int_marker.pose.orientation.y = quat.y;
    int_marker.pose.orientation.z = quat.z;
    int_marker.pose.orientation.w = quat.w;
    server_->setPose(wp_name.str(), int_marker.pose, int_marker.header);
  }
  server_->applyChanges();
}

void WaypointFrame::frameChanged()
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  QString new_frame = ui_->frame_line_edit->text();

  if (new_frame == frame_id_ || new_frame.isEmpty())
    return;

  frame_id_ = new_frame;
  RCLCPP_INFO(node_->get_logger(), "new frame: %s", frame_id_.toStdString().c_str());

  for (auto sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    std::stringstream wp_name;
    wp_name << "waypoint" << sn_it->first;
    visualization_msgs::msg::InteractiveMarker int_marker;
    if (server_->get(wp_name.str(), int_marker)) {
      int_marker.header.frame_id = new_frame.toStdString();
      server_->setPose(wp_name.str(), int_marker.pose, int_marker.header);
    }
  }
  server_->applyChanges();
}

void WaypointFrame::topicChanged()
{
  QString new_topic = ui_->topic_line_edit->text();
  if (new_topic == output_topic_)
    return;

  output_topic_ = new_topic;
  if (!output_topic_.isEmpty() && output_topic_ != "/") {
    wp_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        output_topic_.toStdString(), 1);
  } else {
    wp_pub_.reset();
  }
}

void WaypointFrame::setWpCount(int size)
{
  std::ostringstream ss;
  ss << "Total Wp: " << size;
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(ss.str()));
}

void WaypointFrame::setConfig(QString topic, QString frame, float height)
{
  {
    std::lock_guard<std::mutex> lock(frame_updates_mutex_);
    ui_->topic_line_edit->blockSignals(true);
    ui_->frame_line_edit->blockSignals(true);
    ui_->wp_height_doubleSpinBox->blockSignals(true);
    ui_->topic_line_edit->setText(topic);
    ui_->frame_line_edit->setText(frame);
    ui_->wp_height_doubleSpinBox->setValue(height);
    ui_->topic_line_edit->blockSignals(false);
    ui_->frame_line_edit->blockSignals(false);
    ui_->wp_height_doubleSpinBox->blockSignals(false);
  }
  topicChanged();
  frameChanged();
  heightChanged(height);
}

void WaypointFrame::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  position.x = ui_->x_doubleSpinBox->value();
  position.y = ui_->y_doubleSpinBox->value();
  position.z = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();

  tf2::Quaternion qt;
  qt.setRPY(0, 0, yaw);
  quat.x = qt.x();
  quat.y = qt.y();
  quat.z = qt.z();
  quat.w = qt.w();
}

void WaypointFrame::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quat)
{
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position.x);
  ui_->y_doubleSpinBox->setValue(position.y);
  ui_->z_doubleSpinBox->setValue(position.z);

  tf2::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  ui_->yaw_doubleSpinBox->setValue(tf2::getYaw(qt));

  ui_->x_doubleSpinBox->blockSignals(false);
  ui_->y_doubleSpinBox->blockSignals(false);
  ui_->z_doubleSpinBox->blockSignals(false);
  ui_->yaw_doubleSpinBox->blockSignals(false);
}

void WaypointFrame::setWpLabel(Ogre::Vector3)
{
  ui_->sel_wp_label->setText(QString::fromStdString(selected_marker_name_));
}

double WaypointFrame::getDefaultHeight()
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointFrame::getFrameId()
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointFrame::getOutputTopic()
{
  std::lock_guard<std::mutex> lock(frame_updates_mutex_);
  return output_topic_;
}

}  // namespace waypoint_nav_plugin

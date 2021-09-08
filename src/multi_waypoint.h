#ifndef MULTI_WAYPOINT_TOOL_H
#define MULTI_WAYPOINT_TOOL_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/tool.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

#include "basestation_msgs/Radio.h"

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace rviz_multi_waypoint_plugin
{
// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.

class Waypoint
{
private:
  float radius;
  bool use_orientation;

  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  rviz::Shape* sphere;
  rviz::Arrow* arrow;
  rviz::MovableText* text;

public:
  Waypoint(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
  ~Waypoint();

  void setPosition(Ogre::Vector3 p);
  void setOrientation(Ogre::Quaternion q);
  void setColor(float r, float g, float b, float a = 1.f);
  void useOrientation(bool value);

  Ogre::Vector3 getPosition();
  float getRadius();
  bool getUseOrientation();

  geometry_msgs::Pose getPose();
};

class Line
{
private:
  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  rviz::Line* line;

public:
  Line(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
  ~Line();
  void setPositions(Waypoint* start, Waypoint* end);
  void setColor(float r, float g, float b, float a = 1.f);
};

class Grid
{
private:
  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;

  std::vector<rviz::Line*> lines;

public:
  Grid(Ogre::SceneManager* scene_manager, Ogre::SceneNode* scene_node);
  ~Grid();

  void setPosition(Ogre::Vector3 position);
  void setVisible(bool visible);
};

class MultiWaypointTool : public rviz::Tool
{
  Q_OBJECT
public:
  MultiWaypointTool();
  ~MultiWaypointTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private:
  Ogre::SceneNode* scene_node;
  Waypoint* active_waypoint;
  Line* active_line;
  std::vector<Waypoint*> waypoints;
  std::vector<Line*> lines;
  int active_waypoint_index;
  float z_height;
  bool set_orientation_mode;
  bool space_pressed;
  Grid* grid;
  bool grid_enabled;
  int current_robot_id;

  ros::Publisher path_pub;
  ros::Publisher radio_pub;
  ros::Publisher ugv1_radio_pub;
  ros::Publisher ugv2_radio_pub;
  ros::Publisher ugv3_radio_pub;
  ros::Publisher sp1_radio_pub;
  ros::Publisher sp2_radio_pub;
  ros::Publisher uav1_radio_pub;
  ros::Publisher uav2_radio_pub;
  ros::Publisher uav3_radio_pub;
  ros::Publisher uav4_radio_pub;
  ros::Publisher canary1_radio_pub;
  ros::Publisher canary2_radio_pub;

  ros::Subscriber robot_select_sub;
  ros::Subscriber ugv1_transform_sub;
  ros::Subscriber ugv2_transform_sub;
  ros::Subscriber ugv3_transform_sub;
  ros::Subscriber sp1_transform_sub;
  ros::Subscriber sp2_transform_sub;
  ros::Subscriber uav1_transform_sub;
  ros::Subscriber uav2_transform_sub;
  ros::Subscriber uav3_transform_sub;
  ros::Subscriber uav4_transform_sub;
  ros::Subscriber canary1_transform_sub;
  ros::Subscriber canary2_transform_sub;

  tf::Transform ugv1_transform;
  tf::Transform ugv2_transform;
  tf::Transform ugv3_transform;
  tf::Transform sp1_transform;
  tf::Transform sp2_transform;
  tf::Transform uav1_transform;
  tf::Transform uav2_transform;
  tf::Transform uav3_transform;
  tf::Transform uav4_transform;
  tf::Transform canary1_transform;
  tf::Transform canary2_transform;

  void RobotSelectCallback(const std_msgs::Int32::ConstPtr& robot_select);
  void UGV1TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UGV2TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UGV3TransformCallback(const nav_msgs::OdometryPtr& msg);
  void SP1TransformCallback(const nav_msgs::OdometryPtr& msg);
  void SP2TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UAV1TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UAV2TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UAV3TransformCallback(const nav_msgs::OdometryPtr& msg);
  void UAV4TransformCallback(const nav_msgs::OdometryPtr& msg);
  void CANARY1TransformCallback(const nav_msgs::OdometryPtr& msg);
  void CANARY2TransformCallback(const nav_msgs::OdometryPtr& msg);

  void InitializeTransform(tf::Transform& transform);
  void deleteActive();
  void publishWaypoints();
  void clearWaypoints();
};
// END_TUTORIAL

}  // end namespace rviz_multi_waypoint_plugin

#endif  // PLANT_FLAG_TOOL_H

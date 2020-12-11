#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/console.h>

#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/vector_property.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include <sstream>

#include "multi_waypoint.h"
#include <GL/gl.h>

namespace rviz_multi_waypoint_plugin {

Waypoint::Waypoint(Ogre::SceneManager *scene_manager,
                   Ogre::SceneNode *scene_node)
    : radius(0.5f), scene_manager(scene_manager), scene_node(scene_node),
      use_orientation(false), sphere(NULL), arrow(NULL) {
  text = new rviz::MovableText("........");
  text->setVisible(false);
  scene_node->attachObject(text);
  sphere = new rviz::Shape(rviz::Shape::Sphere, scene_manager, scene_node);
  sphere->setColor(1.f, 0.f, 0.f, 1.f);
  sphere->setScale(Ogre::Vector3(radius, radius, radius));
  arrow = new rviz::Arrow(scene_manager, scene_node, 0.5f, 0.1f, 0.2f, 0.2f);
  arrow->setColor(1.f, 0.f, 0.f, 1.f);
  arrow->getSceneNode()->setVisible(false);
}

Waypoint::~Waypoint() {
  delete sphere;
  delete arrow;
  scene_node->detachObject(text);
  delete text;
}

void Waypoint::setPosition(Ogre::Vector3 p) {
  if (arrow != NULL)
    arrow->setPosition(p);
  if (sphere != NULL) {
    sphere->setPosition(p);

    std::ostringstream out;
    out.precision(1);
    out << std::fixed << p.z;
    text->setCaption(out.str());
    text->setVisible(true);
    text->setCharacterHeight(1.f);
    text->setGlobalTranslation(p);
    text->setColor(Ogre::ColourValue(1.f, 0.5f, 0.f, 1.f));
  }
}

void Waypoint::setOrientation(Ogre::Quaternion q) {
  if (arrow != NULL)
    arrow->setOrientation(q);
}

void Waypoint::setColor(float r, float g, float b, float a) {
  if (sphere != NULL)
    sphere->setColor(r, g, b, a);
  if (arrow != NULL)
    arrow->setColor(r, g, b, a);
}

void Waypoint::useOrientation(bool value) {
  use_orientation = value;

  if (use_orientation)
    arrow->getSceneNode()->setVisible(true);
  else
    arrow->getSceneNode()->setVisible(false);
}
bool Waypoint::getUseOrientation() { return use_orientation; }

Ogre::Vector3 Waypoint::getPosition() {
  if (sphere != NULL)
    return sphere->getPosition();
  if (arrow != NULL)
    return arrow->getPosition();

  return Ogre::Vector3(0, 0, 0);
}

float Waypoint::getRadius() { return radius; }

geometry_msgs::Pose Waypoint::getPose() {
  geometry_msgs::Pose pose;
  Ogre::Vector3 position = getPosition();
  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;

  if (getUseOrientation()) {
    Ogre::Quaternion orientation = arrow->getOrientation();
    pose.orientation.x = orientation.x;
    pose.orientation.y = orientation.y;
    pose.orientation.z = orientation.z;
    pose.orientation.w = orientation.w;
  } else {
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
  }

  return pose;
}
/*
class Test : public Ogre::ManualObject {
public:
  Test(std::string id) :  Ogre::ManualObject(id){}

  bool preRender(Ogre::SceneManager *sm, Ogre::RenderSystem *rsys) override {
    ROS_INFO_STREAM("PRERENDER");
    glLineWidth(5);
    return true;
  }

  bool postRender(Ogre::SceneManager *sm, Ogre::RenderSystem *rsys) override {
    //glLineWidth(1);
    return true;
  }
};
*/

Line::Line(Ogre::SceneManager *scene_manager, Ogre::SceneNode *scene_node)
    : scene_manager(scene_manager), scene_node(scene_node), line(NULL) {

  /*
  Ogre::MaterialPtr m =
  Ogre::MaterialManager::getSingleton().getByName("rviz/waypoint_line");
  ROS_INFO_STREAM("material null: " << (m.isNull()));

  static int id = 0;
  id++;
  //glPointSize(20.f);
  //Test* vertex_data_ = new Test(std::to_string(id));
  Ogre::ManualObject* vertex_data_ = new Ogre::ManualObject(std::to_string(id));
  vertex_data_->estimateVertexCount(6);
  vertex_data_->begin("rviz/waypoint_line",
  Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz");
  vertex_data_->position(Ogre::Vector3(0, 10, 1));
  vertex_data_->normal(Ogre::Vector3(0, 0, 0));
  vertex_data_->colour(Ogre::ColourValue(1, 0, 0, 1));

  vertex_data_->position(Ogre::Vector3(0, 0, 1));
  vertex_data_->normal(Ogre::Vector3(0, 10, 0));
  vertex_data_->colour(Ogre::ColourValue(1, 0, 1, 0));

  vertex_data_->position(Ogre::Vector3(0.3, 10, 1));
  vertex_data_->normal(Ogre::Vector3(0, 0, 0));
  vertex_data_->colour(Ogre::ColourValue(0, 1, 1, 1));



  vertex_data_->position(Ogre::Vector3(0.3, 0, 1));
  vertex_data_->normal(Ogre::Vector3(0, 10, 0));
  vertex_data_->colour(Ogre::ColourValue(1, 0, 1, 0));

  vertex_data_->position(Ogre::Vector3(0, 10, 1));
  vertex_data_->normal(Ogre::Vector3(0, 0, 0));
  vertex_data_->colour(Ogre::ColourValue(1, 0, 0, 1));

  vertex_data_->position(Ogre::Vector3(0, 0, 1));
  vertex_data_->normal(Ogre::Vector3(0, 10, 0));
  vertex_data_->colour(Ogre::ColourValue(0, 1, 0, 0));

  vertex_data_->end();
  vertex_data_->setMaterialName(0, "rviz/waypoint_line", "rviz");


  scene_node->attachObject(vertex_data_);
  vertex_data_->setVisible(true);
  */
}

Line::~Line() { delete line; }

void Line::setPositions(Waypoint *start, Waypoint *end) {
  if (line == NULL) {
    line = new rviz::Line(scene_manager, scene_node);
    line->setColor(0.f, 0.f, 1.f, 1.f);
  }
  line->setPoints(start->getPosition(), end->getPosition());
}

void Line::setColor(float r, float g, float b, float a) {
  line->setColor(r, g, b, a);
}

Grid::Grid(Ogre::SceneManager *scene_manager, Ogre::SceneNode *scene_node)
    : scene_manager(scene_manager), scene_node(scene_node) {
  float radius = 5.f;
  float increment = 1.f;

  for (float x = -radius; x <= radius; x += increment) {
    Ogre::Vector3 start(x, sqrt(-(x * x) + radius * radius), 0.f);
    Ogre::Vector3 stop(x, -sqrt(-(x * x) + radius * radius), 0.f);
    rviz::Line *line = new rviz::Line(scene_manager, scene_node);
    line->setPoints(start, stop);
    line->setColor(1.f, 1.f, 1.f, 0.3f);
    lines.push_back(line);
  }
  for (float x = -radius; x <= radius; x += increment) {
    Ogre::Vector3 start(sqrt(-(x * x) + radius * radius), x, 0.f);
    Ogre::Vector3 stop(-sqrt(-(x * x) + radius * radius), x, 0.f);
    rviz::Line *line = new rviz::Line(scene_manager, scene_node);
    line->setPoints(start, stop);
    line->setColor(1.f, 1.f, 1.f, 1.0f);
    lines.push_back(line);
  }
  setVisible(false);
}

Grid::~Grid() {
  for (int i = 0; i < lines.size(); i++)
    delete lines[i];
  lines.clear();
}

void Grid::setPosition(Ogre::Vector3 position) {
  for (int i = 0; i < lines.size(); i++)
    lines[i]->setPosition(position);
}

void Grid::setVisible(bool visible) {
  for (int i = 0; i < lines.size(); i++)
    lines[i]->setVisible(visible);
}

MultiWaypointTool::MultiWaypointTool()
    : z_height(0.f), set_orientation_mode(false), active_waypoint(NULL),
      active_line(NULL), space_pressed(false), grid(NULL), grid_enabled(true) {
  shortcut_key_ = 'w';
}

MultiWaypointTool::~MultiWaypointTool() {}

void MultiWaypointTool::onInitialize() {
  ros::NodeHandle nh;
  path_pub = nh.advertise<nav_msgs::Path>("waypoints", 1);

  radio_pub = nh.advertise<basestation_msgs::Radio>("/radio_command", 1);
  ugv1_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/ugv1/radio_command", 1);
  ugv2_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/ugv2/radio_command", 1);
  ugv3_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/ugv3/radio_command", 1);
  uav1_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/uav1/radio_command", 1);
  uav2_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/uav2/radio_command", 1);
  uav3_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/uav3/radio_command", 1);
  uav4_radio_pub =
      nh.advertise<basestation_msgs::Radio>("/uav4/radio_command", 1);

  robot_select_sub = nh.subscribe(
      "/robot_select", 10, &MultiWaypointTool::RobotSelectCallback, this);

  scene_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  grid = new Grid(scene_manager_, scene_node);
}

void MultiWaypointTool::activate() {}

void MultiWaypointTool::deactivate() { deleteActive(); }

int MultiWaypointTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (space_pressed) {
    space_pressed = false;
    publishWaypoints();
    clearWaypoints();
    return Render | Finished;
  }
  if (!scene_node)
    return Render;

  if (grid != NULL && active_waypoint != NULL)
    grid->setVisible(grid_enabled);

  z_height += (float)(event.wheel_delta) / 120.f * 0.1f;

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, z_height);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                        event.y, intersection)) {
    // check if the mouse is over a waypoint
    for (int i = 0; i < waypoints.size(); i++) {
      Waypoint *waypoint = waypoints[i];
      Ogre::Vector3 wp_pos = waypoint->getPosition();
      Ogre::Vector3 point;
      Ogre::Plane plane(Ogre::Vector3::UNIT_Z, wp_pos.z);
      if (rviz::getPointOnPlaneFromWindowXY(event.viewport, plane, event.x,
                                            event.y, point)) {
        // check if the mouse is over a waypoint
        if (point.distance(wp_pos) < waypoint->getRadius()) {
          waypoint->setColor(1.f, 1.f, 0.f);

          if (active_waypoint == NULL) {
            // check if an existing waypoint should be moved
            if (event.leftDown()) {
              active_waypoint = waypoint;
              grid->setVisible(grid_enabled);
              z_height = active_waypoint->getPosition().z;
              waypoint->setColor(1.f, 0.f, 0.f);
              return Render;
            }
            // check if an existing waypoint should be deleted
            else if (event.rightDown()) {
              active_waypoint = waypoint;
              deleteActive();
              return Render;
            }
          }
        } else
          waypoint->setColor(0.f, 0.f, 1.f);

        // color the active waypoint red
        if (waypoint == active_waypoint)
          waypoint->setColor(1.f, 0.f, 0.f);

        // update the line end points in case the waypoints moved
        if (i > 0)
          lines[i - 1]->setPositions(waypoint, waypoints[i - 1]);
      }
    }

    // move the active waypoint to the mouse
    if (active_waypoint != NULL) {
      if (set_orientation_mode) {
        Ogre::Vector3 direction = active_waypoint->getPosition() - intersection;
        direction.normalise();
        Ogre::Vector3 up(0, 0, 1);
        Ogre::Vector3 cross = direction.crossProduct(up);
        cross.normalise();
        Ogre::Quaternion q(direction, up, cross);

        active_waypoint->setOrientation(
            q * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
        active_waypoint->useOrientation(true);
      } else {
        if (grid != NULL)
          grid->setPosition(intersection);
        active_waypoint->setPosition(intersection);
        if (active_line != NULL && !waypoints.empty())
          active_line->setPositions(waypoints[waypoints.size() - 1],
                                    active_waypoint);
      }
    }
  }

  if (event.leftDown()) {
    set_orientation_mode = false;
    if (active_waypoint != NULL) {
      if (waypoints.empty() || (!waypoints.empty() && active_line != NULL))
        waypoints.push_back(active_waypoint);
      active_waypoint = NULL;
      grid->setVisible(false);

      if (active_line != NULL) {
        // active_line->setColor(0.f, 1.f, 0.f, 1.f);
        lines.push_back(active_line);
        active_line = NULL;
      }
    } else {
      active_waypoint = new Waypoint(scene_manager_, scene_node);
      active_waypoint->setPosition(intersection);
      grid->setVisible(grid_enabled);
      grid->setPosition(intersection);
      if (!waypoints.empty()) {
        active_line = new Line(scene_manager_, scene_node);
        z_height = waypoints.back()->getPosition().z;
      } else
        z_height = 0.f;
    }
  } else if (event.rightDown()) {
    deleteActive();
  }

  return Render;
}

void MultiWaypointTool::deleteActive() {
  grid->setVisible(false);
  for (int i = 0; i < waypoints.size(); i++) {
    Waypoint *waypoint = waypoints[i];
    if (active_waypoint == waypoint) {
      waypoints.erase(waypoints.begin() + i);
      if (!lines.empty() && i <= lines.size()) {
        if (i == lines.size())
          i--;
        Line *line = lines[i];
        lines.erase(lines.begin() + i);
        delete line;
        if (i - 1 >= 0 && i < waypoints.size())
          lines[i - 1]->setPositions(waypoints[i - 1], waypoints[i]);
      }
      break;
    }
  }
  delete active_waypoint;
  active_waypoint = NULL;
  delete active_line;
  active_line = NULL;
}

void MultiWaypointTool::publishWaypoints() {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = context_->getFixedFrame().toStdString();

  for (int i = 0; i < waypoints.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose = waypoints[i]->getPose();
    path.poses.push_back(pose);
  }

  path_pub.publish(path);

  basestation_msgs::Radio radio_msg;
  radio_msg.message_type =
      basestation_msgs::Radio::MESSAGE_TYPE_DEFINE_WAYPOINT;
  for (int i = 0; i < waypoints.size(); i++) {

    std::stringstream x_stream, y_stream, z_stream;
    x_stream << std::fixed << std::setprecision(2)
             << waypoints[i]->getPosition().x;
    y_stream << std::fixed << std::setprecision(2)
             << waypoints[i]->getPosition().y;
    z_stream << std::fixed << std::setprecision(2)
             << waypoints[i]->getPosition().z;

    std::string x_str = x_stream.str();
    std::string y_str = y_stream.str();
    std::string z_str = z_stream.str();

    radio_msg.data += x_str + "," + y_str + "," + z_str;
    if (i != waypoints.size() - 1) {
      radio_msg.data = radio_msg.data + ";";
    }
  }
  radio_pub.publish(radio_msg);

  switch (current_robot_id) {
  case 1: {
    ugv1_radio_pub.publish(radio_msg);
    break;
  }
  case 2: {
    ugv2_radio_pub.publish(radio_msg);
    break;
  }
  case 3: {
    ugv3_radio_pub.publish(radio_msg);
    break;
  }
  case 4: {
    uav1_radio_pub.publish(radio_msg);
    break;
  }
  case 5: {
    uav2_radio_pub.publish(radio_msg);
    break;
  }
  case 6: {
    uav3_radio_pub.publish(radio_msg);
    break;
  }
  case 7: {
    uav4_radio_pub.publish(radio_msg);
    break;
  }
  default: { ROS_ERROR("Not robot id specified"); }
  }
}

void MultiWaypointTool::RobotSelectCallback(
    const std_msgs::Int32::ConstPtr &robot_select) {
  current_robot_id = robot_select->data;
}

void MultiWaypointTool::clearWaypoints() {
  deleteActive();
  for (int i = 0; i < waypoints.size(); i++)
    delete waypoints[i];
  for (int i = 0; i < lines.size(); i++)
    delete lines[i];
  waypoints.clear();
  lines.clear();
}

int MultiWaypointTool::processKeyEvent(QKeyEvent *event,
                                       rviz::RenderPanel *panel) {
  space_pressed = event->key() == Qt::Key_Space;
  if (active_waypoint != NULL && event->key() == Qt::Key_R) {
    set_orientation_mode = !set_orientation_mode;
    if (set_orientation_mode && active_waypoint->getUseOrientation()) {
      set_orientation_mode = false;
      active_waypoint->useOrientation(false);
    }
  }
  if (event->key() == Qt::Key_E) {
    grid_enabled = !grid_enabled;
    if (active_waypoint != NULL)
      grid->setVisible(grid_enabled);
  }
  if (event->key() == Qt::Key_X)
    clearWaypoints();
  return 0;
}

void MultiWaypointTool::save(rviz::Config config) const {
  config.mapSetValue("Class", getClassId());
}

void MultiWaypointTool::load(const rviz::Config &config) {}

} // end namespace rviz_multi_waypoint_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_multi_waypoint_plugin::MultiWaypointTool,
                       rviz::Tool)
// END_TUTORIAL

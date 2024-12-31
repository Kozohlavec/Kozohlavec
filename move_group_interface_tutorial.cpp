/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "robot_all";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "Linearni_osa_link", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = -1.0;
  visual_tools.publishText(text_pose, "ROBOWASH", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add the first collision object");

  // Přidání prvního kolizního objektu
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "Linearni_osa_link";
  collision_object.id = "C4";

  shapes::Mesh* m = shapes::createMeshFromResource("file:///home/jan-zajicek/robowash_ws/src/robot_t_moveit_config/meshe/C4.stl");
  if (!m)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load mesh resource. Ensure the path is correct.");
    rclcpp::shutdown();
    return 1;
  }

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = 0.5;
  mesh_pose.position.y = 0.0;
  mesh_pose.position.z = -2.5;

  tf2::Quaternion q;
  q.setRPY(0, 0, -1.56); // Původní orientace
  mesh_pose.orientation.x = q.x();
  mesh_pose.orientation.y = q.y();
  mesh_pose.orientation.z = q.z();
  mesh_pose.orientation.w = q.w();

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.addCollisionObjects({collision_object});

  RCLCPP_INFO(LOGGER, "First collision object added.");
  visual_tools.publishText(text_pose, "First Object Added", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();

  // Po stisknutí tlačítka "next" přidání druhého objektu
  visual_tools.prompt("Press 'next' to replace the collision object with a new one");

  // Odstranění předchozího objektu
  collision_object.operation = collision_object.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Přidání nového objektu s posunutím v ose X o -0.5
  collision_object.operation = collision_object.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object);

// Aktualizace pozice objektu
  mesh_pose.position.x -= 0.5; // Posunutí v ose X o -0.5
  collision_object.mesh_poses.clear();
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

// Aplikování nového objektu
  planning_scene_interface.applyCollisionObject(collision_object);

  RCLCPP_INFO(LOGGER, "New collision object added with position offset in X-axis.");
  visual_tools.publishText(text_pose, "New Object Added", rvt::CYAN, rvt::XLARGE);
  visual_tools.trigger();

  // Po stisknutí tlačítka "next" přidání druhého objektu
  visual_tools.prompt("Press 'next' to replace the collision object with a new one");

  // Odstranění předchozího objektu
  collision_object.operation = collision_object.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Přidání nového objektu s upravenou orientací
  tf2::Quaternion q_new;
  q_new.setRPY(0, 0, -1.36); // Upravená orientace
  mesh_pose.orientation.x = q_new.x();
  mesh_pose.orientation.y = q_new.y();
  mesh_pose.orientation.z = q_new.z();
  mesh_pose.orientation.w = q_new.w();

  collision_object.mesh_poses.clear();
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);

  RCLCPP_INFO(LOGGER, "Second collision object added with modified orientation.");
  visual_tools.publishText(text_pose, "Second Object Added", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();

   // Po stisknutí tlačítka "next" přidání třetího objektu
  visual_tools.prompt("Press 'next' to replace the collision object with a new one");

  // Odstranění předchozího objektu
  collision_object.operation = collision_object.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Přidání nového objektu s upravenou orientací
  tf2::Quaternion q_new1;
  q_new1.setRPY(0, 0, -1.76); // Upravená orientace
  mesh_pose.orientation.x = q_new1.x();
  mesh_pose.orientation.y = q_new1.y();
  mesh_pose.orientation.z = q_new1.z();
  mesh_pose.orientation.w = q_new1.w();

  collision_object.mesh_poses.clear();
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);

  RCLCPP_INFO(LOGGER, "Second collision object added with modified orientation.");
  visual_tools.publishText(text_pose, "Second Object Added", rvt::GREEN, rvt::XLARGE);
  visual_tools.trigger();

  // Po stisknutí tlačítka "next" přidání čtvrtého objektu
visual_tools.prompt("Press 'next' to add a new collision object with modified position");

// Přidání nového objektu s posunutím v ose X o -0.5
collision_object.operation = collision_object.REMOVE;
planning_scene_interface.applyCollisionObject(collision_object);

// Aktualizace pozice objektu
mesh_pose.position.x -= 0.5; // Posunutí v ose X o -0.5
collision_object.mesh_poses.clear();
collision_object.mesh_poses.push_back(mesh_pose);
collision_object.operation = collision_object.ADD;

// Aplikování nového objektu
planning_scene_interface.applyCollisionObject(collision_object);

RCLCPP_INFO(LOGGER, "New collision object added with position offset in X-axis.");
visual_tools.publishText(text_pose, "New Object Added", rvt::CYAN, rvt::XLARGE);
visual_tools.trigger();


  rclcpp::shutdown();
  return 0;
}

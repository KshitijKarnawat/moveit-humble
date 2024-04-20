#include <memory>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

using moveit::planning_interface::MoveGroupInterface;


void move_to_home(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto const joint_state = []{
    sensor_msgs::msg::JointState msg;
    msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    msg.position = {0.0, 0.0, 0.0, 0.0, 0.0, 1.5708, 0.785398};
    return msg;
  }();

  move_group_interface.setJointValueTarget(joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

}

void move_to_pick(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto const joint_state = []{
    sensor_msgs::msg::JointState msg;
    msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    msg.position = {0.0, 0.45, 0.0, -0.75, 0.0, 1.3, 0.0};
    return msg;
  }();

  move_group_interface.setJointValueTarget(joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

}

void move_to_place(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto const joint_state = []{
    sensor_msgs::msg::JointState msg;
    msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    msg.position = {1.5, 0.75, 0.0, -1.0, 0.0, 1.5, 0.0};
    return msg;
  }();

  move_group_interface.setJointValueTarget(joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

}

void open_hand(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto const joint_state = []{
    sensor_msgs::msg::JointState msg;
    msg.name = {"panda_finger_joint1"};
    msg.position = {0.04};
    return msg;
  }();

  move_group_interface.setJointValueTarget(joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

}

void close_hand(moveit::planning_interface::MoveGroupInterface &move_group_interface){
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto const joint_state = []{
    sensor_msgs::msg::JointState msg;
    msg.name = {"panda_finger_joint1"};
    msg.position = {0.0};
    return msg;
  }();

  move_group_interface.setJointValueTarget(joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

}


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pick_place");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_arm = MoveGroupInterface(node, "panda_arm");
  auto move_group_hand = MoveGroupInterface(node, "hand");

  // Go to the home position
  move_to_home(move_group_arm);
  RCLCPP_INFO(logger, "Reached home position!");
  sleep(3);

  // Open the hand
  open_hand(move_group_hand);
  RCLCPP_INFO(logger, "opened hand!");
  sleep(3);

  // Go to the pick position
  move_to_pick(move_group_arm);
  RCLCPP_INFO(logger, "Reached pick position!");
  sleep(3);

    // Close the hand
  close_hand(move_group_hand);
  RCLCPP_INFO(logger, "closed hand!");
  sleep(3);

  // Go to the place position
  move_to_place(move_group_arm);
  RCLCPP_INFO(logger, "Reached place position!");
  sleep(3);

  // Open the hand
  open_hand(move_group_hand);
  RCLCPP_INFO(logger, "opened hand!");
  sleep(3);

  // Go to the home position
  move_to_home(move_group_arm);
  RCLCPP_INFO(logger, "Reached home position!");

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

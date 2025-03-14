#ifndef PINOCCHIO_TIAGO_NODE_HPP_
#define PINOCCHIO_TIAGO_NODE_HPP_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Node that uses Pinocchio to perform various tasks on the TIAGo robot model
 */
class PinocchioTiagoNode : public rclcpp::Node
{
public:
  /// @brief Constructor
  PinocchioTiagoNode();

private:

  /// @brief Check for collisions between two links
  void checkCollisions(const std::string & link1, const std::string & link2);

  /// @brief Create the Pinocchio model from a URDF file
  void createPinocchioModel(const std::string & urdf_path);

  /// @brief Compute the Jacobian matrix for a specific frame
  void getJacobian();

  /// @brief Perform forward kinematics and print the end effector position and orientation
  void performForwardKinematics();

  /// @brief Callback for the robot description topic
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief Joint configuration vector
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(model_.nq);

  /// @brief Flag indicating if the model was created
  bool model_created_{false};

  /// @brief Pinocchio data structures
  pinocchio::Data data_;

  /// @brief Pinocchio model
  pinocchio::Model model_;

  /// @brief Pinocchio collision model
  pinocchio::GeometryModel collision_model_;

  /// @brief Pinocchio collision data
  pinocchio::GeometryData collision_data_;

  /// @brief Frame index for the end effector
  pinocchio::FrameIndex arm_7_link;

  /// @brief Subscription to the robot description topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // PINOCCHIO_TIAGO_NODE_HPP_

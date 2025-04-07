#ifndef PAYLOAD_VISUALIZATION_AND_METRICS_HPP_
#define PAYLOAD_VISUALIZATION_AND_METRICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>  // Forward declarations
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>  // Required for numeric_limits
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

const double GRAVITY_ACCELERATION = 9.80665;

/**
 * @brief Node that uses Pinocchio to perform various tasks on the TIAGo robot model
 */
class PayloadVisualizationAndMetricsNode : public rclcpp::Node
{
public:
  /// @brief Constructor
  PayloadVisualizationAndMetricsNode();

private:
  /// @brief Calculate max downward force
  bool calculateMaxDownwardForce();

  /// @brief Calculate Torque and Jacobian
  bool getGravityTorques();

  /// @brief Create the Pinocchio model from a URDF file
  bool createPinocchioModel(const std::string & urdf_path);

  /// @brief Extract kinematic and dynamic properties from the model
  void extractKinematicsDynamicsProperties();

  /// @brief Compute the Jacobian matrix for a specific frame
  bool getJacobian();

  /// @brief Perform forward kinematics and print the end effector position and orientation
  bool performForwardKinematics();

  /// @brief print current joint state
  void printCurrentQ();

  /// @brief Callback for the robot description topic
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief Callback for the joint state topic
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief Setup the robot model
  bool SetUp(const std_msgs::msg::String::SharedPtr msg);

  /// @brief Setup variables
  bool SetUpVariables();

  /// @brief Joint configuration vector
  Eigen::VectorXd currentQ_ = Eigen::VectorXd::Zero(model_.nv);

  /// @brief Joint configuration vector
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(model_.nq);

  /// @brief Torque vector
  Eigen::VectorXd tau_gravity_ = Eigen::VectorXd::Zero(model_.nv);

  /// @brief Jacobian matrix
  Eigen::MatrixXd J;

  /// @brief Dimension of the configuration vector representation
  int nq_;

  /// @brief Number of joints
  int njoints_;

  /// @brief Number of frames
  int nframes_;

  /// @brief Dimension of the velocity vector space
  int nv_;

  /// @brief Maximum payload mass
  double latest_max_payload_mass_ = 0.0;  // Store the result

  /// @brief Flag indicating if the model was created
  bool model_created_{false};

  /// @brief Pinocchio data structures
  pinocchio::Data data_;

  /// @brief Pinocchio model
  pinocchio::Model model_;

  /// @brief Frame index for the end effector
  pinocchio::FrameIndex arm_7_link;

  /// @brief Subscription to the robot description topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  /// @brief Subscription for JointState messages
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

#endif  // PAYLOAD_VISUALIZATION_AND_METRICS_HPP_

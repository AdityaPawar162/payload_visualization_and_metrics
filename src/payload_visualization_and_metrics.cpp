#include "gsoc_payload_visualization_and_metrics/payload_visualization_and_metrics.hpp"

/**
 * @brief Node that uses Pinocchio to perform various tasks on the TIAGo robot model
 */
PayloadVisualizationAndMetricsNode::PayloadVisualizationAndMetricsNode() : Node("payload_visualization_and_metrics_node")
{
  // Create subscription to robot_description topic
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), // Use transient local for robot_description
    std::bind(&PayloadVisualizationAndMetricsNode::robot_description_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,std::bind(&PayloadVisualizationAndMetricsNode::joint_state_callback, this, std::placeholders::_1)); 
  
  RCLCPP_INFO(this->get_logger(), "Pinocchio TIAGo Node started. Waiting for robot description...");
}

/**
 * @brief Callback for the robot description topic
 * @param msg Message containing the URDF model of the robot
 */
void PayloadVisualizationAndMetricsNode::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received robot description. Creating Pinocchio model...");

  try
  {
    // Setup the robot model
    if(!SetUp(msg))
    {
      RCLCPP_ERROR(this->get_logger(), "Error creating Pinocchio model");
      return;
    }

    if(!SetUpVariables())
    {
      RCLCPP_ERROR(this->get_logger(), "Error setting up variables");
      return;
    }

    // Extract Kinematic and dynamic properties
    // extractKinematicsDynamicsProperties();
    // Perform tests
    // performForwardKinematics();
    // getJacobian();
    // checkCollisions("arm_1_link", "arm_4_link");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
  }
}

void PayloadVisualizationAndMetricsNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Check if model was created before processing joint states
  if(!model_created_)
  {
    return;
  }

  try {
    // Create a mapping from joint names to their position in the message
    std::unordered_map<std::string, size_t> joint_name_to_idx;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_name_to_idx[msg->name[i]] = i;
    }
    
    // Make sure currentQ_ is properly sized
    if (currentQ_.size() != nq_) {
      currentQ_ = Eigen::VectorXd::Zero(nq_);
    }
    
    // Debug output
    RCLCPP_INFO(this->get_logger(), "Received joint states with %zu positions", msg->position.size());
    
    // Update each joint in the model that exists in the message
    for (size_t i = 1; i < njoints_; ++i) {  // Skip joint 0 (the universe)
      const std::string& joint_name = model_.names[i];
      auto it = joint_name_to_idx.find(joint_name);
      
      if (it != joint_name_to_idx.end()) {
        // Get the joint model and its position in the configuration vector
        const pinocchio::JointIndex joint_id = i;
        const pinocchio::JointIndex joint_config_start = model_.joints[joint_id].idx_q();
        
        // For simple joints (1 DoF), just copy the value
        if (model_.joints[joint_id].nq() == 1 && joint_config_start < currentQ_.size() && it->second < msg->position.size()) {
          currentQ_[joint_config_start] = msg->position[it->second];
        }
      }
    }
    // Print the current joint configuration
    printCurrentQ();
    
    // Now calculate torques and Jacobian using the updated configuration
    if(!performForwardKinematics())
    {
      RCLCPP_ERROR(this->get_logger(), "Error performing forward kinematics");
      return;
    }
    
    if(!getGravityTorques())
    {
      RCLCPP_ERROR(this->get_logger(), "Error calculating torque");
      return;
    }

    if(!getJacobian())
    {
      RCLCPP_ERROR(this->get_logger(), "Error calculating Jacobian");
      return;
    }

    if (!calculateMaxDownwardForce()) {
      RCLCPP_ERROR(this->get_logger(), "Error calculating max payload in callback.");
      return;
    }

  } catch(const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in joint state callback: %s", e.what());
  }
}

/**
 * @brief Calculate the torque and Jacobian for the current joint positions
 */
bool PayloadVisualizationAndMetricsNode::getGravityTorques()
{  
  if(!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping torque calculation.");
    return false;
  }
  /// @brief Joint velocity vector
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv_);

  /// @brief Joint acceleration vector
  Eigen::VectorXd a = Eigen::VectorXd::Zero(nv_);

  // Calculate torques using RNEA (gravity compensation)
  tau_gravity_ = pinocchio::rnea(model_, data_, currentQ_, v, a);
  
  RCLCPP_INFO(this->get_logger(), "Joint torques at current position:");
  for (size_t i = 0; i < std::min(static_cast<size_t>(nv_), static_cast<size_t>(10)); ++i) {
    RCLCPP_INFO(this->get_logger(), "Joint %zu torque: %f Nm", i, tau_gravity_[i]);
  }
  
  return true;

}

/**
 * @brief Calculate Max Downward Force 
 */
bool PayloadVisualizationAndMetricsNode::calculateMaxDownwardForce()
{
  if(!model_created_)
  {
  RCLCPP_INFO(this->get_logger(), "Model not created. Skipping max downward force calculation.");
  return false;
  }
  
  const Eigen::VectorXd& tau_max_limit = model_.effortLimit;
  Eigen::VectorXd tau_available_positive(nv_);
  Eigen::VectorXd tau_available_negative(nv_);

  for(int i = 0; i < nv_;++i)
  {
    tau_available_positive[i] = tau_max_limit[i] - tau_gravity_[i];
    tau_available_negative[i] = -tau_max_limit[i] - tau_gravity_[i];

  }

  pinocchio::Force F_payload_unit = pinocchio::Force::Zero();
  F_payload_unit.linear()(2) = -1.0;

  // tau = J^T * F
  Eigen::VectorXd tau_per_unit_force = J.transpose()*F_payload_unit.toVector();
  double min_P_max_i = std::numeric_limits<double>::infinity();
  int limiting_joint_idx_v = -1; 
  
  for (int i = 0; i < nv_; ++i) {
    double P_max_i = std::numeric_limits<double>::infinity();
    double tau_needed = tau_per_unit_force[i];
    const double tolerance = 1e-9; // Tolerance for near-zero checks

    if (std::abs(tau_needed) < tolerance) {
      // This joint is not significantly affected by the payload force in this direction.
      continue;
    }

    if (tau_needed > tolerance) { // Payload requires positive torque from joint i
      if (tau_available_positive[i] <= tolerance) {
        P_max_i = 0.0; // No positive torque available
      } else {
        P_max_i = tau_available_positive[i] / tau_needed;
      }
    } else { // Payload requires negative torque from joint i (tau_needed < -tolerance)
      if (tau_available_negative[i] >= -tolerance) {
        P_max_i = 0.0; 
      } else {
        P_max_i = tau_available_negative[i] / tau_needed;
      }
    }
    
    if (P_max_i >= 0 && P_max_i < min_P_max_i) {
      min_P_max_i = P_max_i;
      limiting_joint_idx_v = i;
    }
  }
  
  if (min_P_max_i == std::numeric_limits<double>::infinity()) {
    RCLCPP_WARN(this->get_logger(), "No joint appears to limit the payload. Calculation might be inaccurate or payload direction has no effect.");
    latest_max_payload_mass_ = std::numeric_limits<double>::infinity();
  } else {
     min_P_max_i = std::max(0.0, min_P_max_i);

    double max_payload_force = min_P_max_i;
    latest_max_payload_mass_ = max_payload_force / GRAVITY_ACCELERATION;

    RCLCPP_INFO(this->get_logger(), "Max Additional Payload Mass (Downward): %.4f kg (Force Limit: %.4f N)",
                latest_max_payload_mass_, max_payload_force);

    if (limiting_joint_idx_v != -1) {
         RCLCPP_INFO(this->get_logger(), "Limiting Joint (Velocity Index): %d", limiting_joint_idx_v);
    } else if (latest_max_payload_mass_ > 0) {
          RCLCPP_WARN(this->get_logger(), "Payload limit found, but couldn't identify a specific limiting joint index.");
    }
  }

return true;

}


/**
 * @brief Create the Pinocchio model from a URDF file
 * @param urdf_path Path to the URDF file
 */
bool PayloadVisualizationAndMetricsNode::SetUp(const std_msgs::msg::String::SharedPtr msg)
{
  // Save URDF to a temporary file
  std::string temp_file_path = "/tmp/tiago_robot_temp.urdf";
  std::ofstream urdf_file(temp_file_path);
  urdf_file << msg->data;
  urdf_file.close();

  // Load URDF model
  pinocchio::urdf::buildModel(temp_file_path, model_);
  data_ = pinocchio::Data(model_);
  
  currentQ_ = Eigen::VectorXd::Zero(nq_);

  // Set model_created_ flag to true
  model_created_ = true;

  return model_created_;
}

/**
 * @brief Setup variables for the model
 */
bool PayloadVisualizationAndMetricsNode::SetUpVariables()
{
  nq_           = model_.nq;                        // Dimension of the configuration vector representation
  njoints_      = model_.njoints;                   // Number of joints
  nframes_      = model_.nframes;                   // Number of frames
  nv_           = model_.nv;                        // Dimension of the velocity vector space
  arm_7_link    = model_.getFrameId("arm_7_link");  // Frame index for the end effector
  J.resize(6, nv_);                                 // Resize Jacobian matrix to 6xnv
  J.setZero();                                      // Initialize Jacobian matrix to zero
  return true;
}

/**
 * @brief Print the current joint configuration
 */
void PayloadVisualizationAndMetricsNode::printCurrentQ()
{
  if(!model_created_)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot print currentQ_: model not created");
    return;
  }

  if (!model_created_ || currentQ_.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot print currentQ_: model not created or vector is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully created Pinocchio model:");
  RCLCPP_INFO(this->get_logger(), "  Number of joints: %d", njoints_);
  RCLCPP_INFO(this->get_logger(), "  Number of frames: %d", nframes_);
  RCLCPP_INFO(this->get_logger(), "  Number of DOFs: %d", nv_);


  std::stringstream ss;
  ss << "currentQ_ (" << currentQ_.size() << " elements): [";
  for (int i = 0; i < currentQ_.size(); ++i)
  {
    ss << currentQ_[i];
    if (i < currentQ_.size() - 1)
      ss << ", ";
  }
  ss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}


/**
 * @brief Extract kinematic and dynamic properties from the model
 */
void PayloadVisualizationAndMetricsNode::extractKinematicsDynamicsProperties()
{ 
  // Print information about the model
  // Print Joint Limits and Names
  RCLCPP_INFO(this->get_logger(), "Joint [Position Limits] [Velocity Limits] [Torque Limits]:");
  for (size_t i = 1; i < njoints_; i++)
  {
    
    const std::string & joint_name = model_.names[i];
    
    const pinocchio::JointModel & joint_model = model_.joints[i];
    const int joint_config_start = joint_model.idx_q();
    const int joint_velocity_start = joint_model.idx_v();

    RCLCPP_INFO(this->get_logger(),"Joint mode idx_q: %d", joint_config_start);
    for (int dof =0; dof < joint_model.nq(); dof++)
    {
      int config_idx = joint_config_start + dof;
      RCLCPP_INFO(this->get_logger(), "Joint %zu (%s) DoF %d: [%f, %f]", 
                 i, joint_name.c_str(), dof,
                 model_.lowerPositionLimit[config_idx], 
                 model_.upperPositionLimit[config_idx]);                
    }

    for(int dof = 0; dof < joint_model.nv(); dof++)
    {
      int velocity_idx = joint_velocity_start + dof;
      RCLCPP_INFO(this->get_logger(), "Joint %zu (%s) DoF %d velocity limit: %f", 
              i, joint_name.c_str(), dof,
              model_.velocityLimit[velocity_idx]);

      RCLCPP_INFO(this->get_logger(), "Joint %zu (%s) DoF %d torque/effort limit: %f", 
              i, joint_name.c_str(), dof,
              model_.effortLimit[velocity_idx]);
    }   
  }

}

/**
 * @brief Perform forward kinematics and print the end effector position and orientation
 */
bool PayloadVisualizationAndMetricsNode::performForwardKinematics()
{
  // Check if model was created
  if (!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping forward kinematics test.");
    return false;
  }

  // Perform forward kinematics
  pinocchio::forwardKinematics(model_, data_, currentQ_);

  // Compute the end-effector position and orientation
  pinocchio::updateFramePlacements(model_, data_);

  return true;
}

/**
 * @brief Compute the Jacobian matrix for a specific frame
 */
bool PayloadVisualizationAndMetricsNode::getJacobian()
{
  // Check if model was created
  if (!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping Jacobian test.");
    return false;
  }

    // Compute Jacobian (6xnv matrix for position and orientation)
    
    pinocchio::computeFrameJacobian(model_, data_, currentQ_, arm_7_link, pinocchio::LOCAL_WORLD_ALIGNED, J);
    
    RCLCPP_INFO(this->get_logger(), "Jacobian norm at end-effector: %f", J.norm());
    
    RCLCPP_INFO(
      this->get_logger(),
      "Jacobian:\n%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n"
      "%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n"
      "%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",
      J(0, 0), J(0, 1), J(0, 2), J(0, 3),J(0, 4), J(0, 5),J(0, 6), J(0, 7), J(0, 8), J(0, 9),J(0, 10), J(0, 11), J(0,12),J(0,13),
      J(1, 0), J(1, 1),J(1, 2), J(1, 3), J(1, 4), J(1, 5),J(1, 6), J(1, 7),J(1, 8), J(1, 9), J(1, 10), J(1, 11), J(1,12),J(1,13),
      J(2, 0), J(2, 1), J(2, 2), J(2, 3),J(2, 4), J(2, 5),J(2, 6), J(2, 7), J(2, 8), J(2, 9), J(2, 10), J(2, 11), J(2,12),J(2,13),
      J(3, 0), J(3, 1),J(3, 2), J(3, 3), J(3, 4), J(3, 5),J(3, 6), J(3, 7), J(3, 8), J(3, 9), J(3, 10), J(3, 11), J(3,12),J(3,13),
      J(4, 0), J(4, 1), J(4, 2), J(4, 3),J(4, 4), J(4, 5),J(4, 6), J(4, 7), J(4, 8), J(4, 9), J(4, 10), J(4, 11), J(4,12),J(4,13),
      J(5, 0), J(5, 1),J(5, 2), J(5, 3), J(5, 4), J(5, 5),J(5, 6), J(5, 7), J(5, 8), J(5, 9), J(5, 10), J(5, 11), J(5,12),J(5,13));
    
    return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PayloadVisualizationAndMetricsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

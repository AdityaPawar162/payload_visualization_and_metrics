#include "payload_visualization_and_metrics/pinocchio_task.hpp"

/**
 * @brief Node that uses Pinocchio to perform various tasks on the TIAGo robot model
 */
PinocchioTiagoNode::PinocchioTiagoNode() : Node("pinocchio_tiago_node")
{
  // Create subscription to robot_description topic
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "robot_description", 10,
    std::bind(&PinocchioTiagoNode::robot_description_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Pinocchio TIAGo Node started. Waiting for robot description...");
}

/**
 * @brief Callback for the robot description topic
 * @param msg Message containing the URDF model of the robot
 */
void PinocchioTiagoNode::robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received robot description. Creating Pinocchio model...");

  try
  {
    // Save URDF to a temporary file
    std::string temp_file_path = "/tmp/tiago_robot_temp.urdf";
    std::ofstream urdf_file(temp_file_path);
    urdf_file << msg->data;
    urdf_file.close();

    // Create Pinocchio model
    createPinocchioModel(temp_file_path);

    // Perform tests
    performForwardKinematics();
    getJacobian();
    checkCollisions("arm_1_link", "arm_4_link");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
  }
}

/**
 * @brief Create the Pinocchio model from a URDF file
 * @param urdf_path Path to the URDF file
 */
void PinocchioTiagoNode::createPinocchioModel(const std::string & urdf_path)
{
  // Load URDF model
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  // Initialize geometry model and data
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, collision_model_);
  collision_model_.addAllCollisionPairs();
  collision_data_ = pinocchio::GeometryData(collision_model_);

  // Get the frame index for the end effector
  arm_7_link = model_.getFrameId("arm_7_link");

  // Print information about the model
  RCLCPP_INFO(this->get_logger(), "Successfully created Pinocchio model:");
  RCLCPP_INFO(this->get_logger(), "  Number of joints: %d", model_.njoints);
  RCLCPP_INFO(this->get_logger(), "  Number of frames: %d", model_.nframes);
  RCLCPP_INFO(this->get_logger(), "  Number of DOFs: %d", model_.nv);

  // Print joint names with their IDs
  RCLCPP_INFO(this->get_logger(), "Joints:");
  for (size_t i = 1; i < model_.joints.size(); ++i)
  {
    const std::string & joint_name = model_.names[i];
    RCLCPP_INFO(this->get_logger(), "  %zu: %s", i, joint_name.c_str());
  }

  // Set model_created_ flag to true
  model_created_ = true;
}

/**
 * @brief Perform forward kinematics and print the end effector position and orientation
 */
void PinocchioTiagoNode::performForwardKinematics()
{
  // Check if model was created
  if (!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping forward kinematics test.");
    return;
  }

  // Set random configuration
  q_ = Eigen::VectorXd::Random(model_.nq);
  // q_ = pinocchio::neutral(model_);

  // Perform forward kinematics
  pinocchio::framesForwardKinematics(model_, data_, q_);

  // Retrieve the transformation matrix
  const pinocchio::SE3 & transform = data_.oMf[arm_7_link];

  // Print end effector position
  RCLCPP_INFO(
    this->get_logger(), "End effector position: [%f, %f, %f]", transform.translation()[0],
    transform.translation()[1], transform.translation()[2]);

  // Print end effector orientation (rotation matrix)
  RCLCPP_INFO(
    this->get_logger(), "End effector rotation matrix:\n%f %f %f\n%f %f %f\n%f %f %f",
    transform.rotation()(0, 0), transform.rotation()(0, 1), transform.rotation()(0, 2),
    transform.rotation()(1, 0), transform.rotation()(1, 1), transform.rotation()(1, 2),
    transform.rotation()(2, 0), transform.rotation()(2, 1), transform.rotation()(2, 2));
}

/**
 * @brief Compute the Jacobian matrix for a specific frame
 */
void PinocchioTiagoNode::getJacobian()
{
  // Check if model was created
  if (!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping Jacobian test.");
    return;
  }

  // Create Jacobian matrix
  Eigen::MatrixXd arm_7_link_J(6, model_.nv);

  // Compute the Jacobian
  pinocchio::computeFrameJacobian(model_, data_, q_, arm_7_link, arm_7_link_J);

  // Print the Jacobian
  RCLCPP_INFO(
    this->get_logger(),
    "Jacobian:\n%f %f %f %f %f %f\n%f %f %f %f %f %f\n%f %f %f %f %f %f\n%f %f %f %f %f %f\n%f %f "
    "%f %f %f %f\n%f %f %f %f %f %f",
    arm_7_link_J(0, 0), arm_7_link_J(0, 1), arm_7_link_J(0, 2), arm_7_link_J(0, 3),
    arm_7_link_J(0, 4), arm_7_link_J(0, 5), arm_7_link_J(1, 0), arm_7_link_J(1, 1),
    arm_7_link_J(1, 2), arm_7_link_J(1, 3), arm_7_link_J(1, 4), arm_7_link_J(1, 5),
    arm_7_link_J(2, 0), arm_7_link_J(2, 1), arm_7_link_J(2, 2), arm_7_link_J(2, 3),
    arm_7_link_J(2, 4), arm_7_link_J(2, 5), arm_7_link_J(3, 0), arm_7_link_J(3, 1),
    arm_7_link_J(3, 2), arm_7_link_J(3, 3), arm_7_link_J(3, 4), arm_7_link_J(3, 5),
    arm_7_link_J(4, 0), arm_7_link_J(4, 1), arm_7_link_J(4, 2), arm_7_link_J(4, 3),
    arm_7_link_J(4, 4), arm_7_link_J(4, 5), arm_7_link_J(5, 0), arm_7_link_J(5, 1),
    arm_7_link_J(5, 2), arm_7_link_J(5, 3), arm_7_link_J(5, 4), arm_7_link_J(5, 5));
}

/**
 * @brief Check for collisions between two links
 * @param link1 Name of the first link
 * @param link2 Name of the second link
 */
void PinocchioTiagoNode::checkCollisions(const std::string & link1, const std::string & link2)
{
  // Check if model was created
  if (!model_created_)
  {
    RCLCPP_ERROR(this->get_logger(), "Model not created. Skipping collision check.");
    return;
  }

  // Find geometry objects corresponding to these links
  int geom1_id = -1, geom2_id = -1;
  for (size_t i = 0; i < collision_model_.geometryObjects.size(); ++i)
  {
    if (collision_model_.geometryObjects[i].name.find(link1) != std::string::npos)
    {
      geom1_id = i;
    }
    if (collision_model_.geometryObjects[i].name.find(link2) != std::string::npos)
    {
      geom2_id = i;
    }
  }

  if (geom1_id >= 0 && geom2_id >= 0)
  {
    // Find the pair index, if it exists
    pinocchio::PairIndex pair_id = -1;
    for (size_t i = 0; i < collision_model_.collisionPairs.size(); ++i)
    {
      const auto & cp = collision_model_.collisionPairs[i];
      if (
        (cp.first == geom1_id && cp.second == geom2_id) ||
        (cp.first == geom2_id && cp.second == geom1_id))
      {
        pair_id = i;
        break;
      }
    }

    if (pair_id >= 0)
    {
      // Compute collision for this specific pair
      bool isColliding = pinocchio::computeCollision(collision_model_, collision_data_, pair_id);

      RCLCPP_INFO(
        this->get_logger(), "Specific check between %s and %s: %s", link1.c_str(), link2.c_str(),
        isColliding ? "COLLISION" : "NO COLLISION");
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(), "No collision pair found between %s and %s", link1.c_str(),
        link2.c_str());
    }
  }
  else
  {
    RCLCPP_WARN(
      this->get_logger(), "Could not find geometry objects for links %s and/or %s", link1.c_str(),
      link2.c_str());
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PinocchioTiagoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
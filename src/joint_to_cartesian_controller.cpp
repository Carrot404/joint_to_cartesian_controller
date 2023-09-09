/*
 * @Description: joint to cartesian controller
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2023-09-08 21:06:43
 * @LastEditTime: 2023-09-09 16:14:30
 */

#include "joint_to_cartesian_controller/joint_to_cartesian_controller.hpp"

#include "Eigen/Geometry"
#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/time.h"
#include "tf2_kdl/tf2_kdl.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/helpers.hpp"

namespace
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_to_cartesian_controller");
}

namespace joint_to_cartesian_controller
{
  controller_interface::InterfaceConfiguration JointToCartesianController::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration JointToCartesianController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(m_joint_names.size() * m_state_interface_types.size());
    for (const auto &type : m_state_interface_types)
    {
      for (const auto &joint_name : m_joint_names)
      {
        conf.names.push_back(joint_name + std::string("/").append(type));
      }
    }
    return conf;
  }

  controller_interface::CallbackReturn JointToCartesianController::on_init()
  {
    try
    {
      // Create the parameter listener and get the parameters
      m_param_listener = std::make_shared<joint_to_cartesian_controller::ParamListener>(get_node());
      m_params = m_param_listener->get_params();

      RCLCPP_INFO(LOGGER, "Loading JointToCartesianController with tf_prefix: %s", m_params.tf_prefix.c_str());
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    m_initialized = true;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointToCartesianController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    if (m_configured)
    {
      RCLCPP_WARN(LOGGER, "Cannot configure, controller is already configured");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    if (!m_param_listener)
    {
      RCLCPP_ERROR(LOGGER, "Error encountered during configure stage, parameter listener not initialized");
      return controller_interface::CallbackReturn::ERROR;
    }

    // update the dynamic map parameters
    m_param_listener->refresh_dynamic_parameters();
    // get parameters from the listener in case they were updated
    m_params = m_param_listener->get_params();

    m_tf_prefix = m_params.tf_prefix;

    m_robot_base_link = m_params.robot_base_link;
    if (m_robot_base_link.empty())
    {
      RCLCPP_ERROR(LOGGER, "robot_base_link is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    m_robot_base_link = m_tf_prefix + m_robot_base_link;

    m_end_effector_link = m_params.end_effector_link;
    if (m_end_effector_link.empty())
    {
      RCLCPP_ERROR(LOGGER, "end_effector_link is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    m_end_effector_link = m_tf_prefix + m_end_effector_link;

    m_robot_description = get_node()->get_parameter("robot_description").as_string();
    if (m_robot_description.empty())
    {
      RCLCPP_ERROR(LOGGER, "robot_description is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    urdf::Model robot_model;
    if (!robot_model.initString(m_robot_description))
    {
      RCLCPP_ERROR(LOGGER, "Unable to initialize urdf::Model from robot_description");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(LOGGER, "Reading joints and links from URDF");

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
    {
      RCLCPP_ERROR(LOGGER, "Failed to extract kdl tree from xml robot description");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!kdl_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
    {
      RCLCPP_ERROR(LOGGER, "Could not find chain from %s to %s", m_robot_base_link.c_str(), m_end_effector_link.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(LOGGER, "Loaded KDL chain from %s to %s with %d joints", m_robot_base_link.c_str(), m_end_effector_link.c_str(), m_robot_chain.getNrOfJoints());

    m_joint_names = m_params.joints;
    if (m_joint_names.empty())
    {
      RCLCPP_INFO(LOGGER, "No joints in params is specified, using default joints");
      for (std::vector<KDL::Segment>::const_iterator it = m_robot_chain.segments.begin(); it != m_robot_chain.segments.end(); ++it)
      {
        if (it->getJoint().getType() != KDL::Joint::None)
        {
          RCLCPP_INFO(LOGGER, "Adding joint %s", it->getJoint().getName().c_str());
          m_joint_names.push_back(it->getJoint().getName());
        }
      }
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Joints in params is specified");

      for (std::vector<std::string>::iterator it = m_joint_names.begin(); it != m_joint_names.end(); ++it)
      {
        // add the tf_prefix to the joint name
        *it = m_tf_prefix + *it;
        // TODO: check if the m_joint_names is in the chain
      }
    }

    m_state_interface_types = m_params.state_interfaces;
    if (m_state_interface_types.empty())
    {
      RCLCPP_INFO(LOGGER, "No state interfaces in params is specified, using default state interfaces");
      m_state_interface_types.push_back(hardware_interface::HW_IF_POSITION);
      m_state_interface_types.push_back(hardware_interface::HW_IF_VELOCITY);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "State interfaces in params is specified");
      // check if the state interface is valid or not
      for (std::vector<std::string>::iterator it = m_state_interface_types.begin(); it != m_state_interface_types.end(); ++it)
      {
        if (*it != hardware_interface::HW_IF_POSITION && *it != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_ERROR(LOGGER, "State interface %s is not valid", it->c_str());
          return controller_interface::CallbackReturn::ERROR;
        }
      }
    }

    m_number_of_joints = m_joint_names.size();

    m_current_positions.data = Eigen::VectorXd::Zero(m_number_of_joints);
    m_current_velocities.data = Eigen::VectorXd::Zero(m_number_of_joints);
    m_last_positions.data = Eigen::VectorXd::Zero(m_number_of_joints);
    m_last_velocities.data = Eigen::VectorXd::Zero(m_number_of_joints);

    m_fk_pos_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(m_robot_chain);
    m_fk_vel_solver = std::make_shared<KDL::ChainFkSolverVel_recursive>(m_robot_chain);

    m_feedback_pose_publisher = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            m_params.pose_pub_name, rclcpp::SystemDefaultsQoS()));
    m_feedback_twist_publisher = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
        get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
            m_params.twist_pub_name, rclcpp::SystemDefaultsQoS()));

    m_configured = true;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointToCartesianController::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    if (!m_configured)
    {
      RCLCPP_WARN(LOGGER, "Cannot activate, controller is not configured");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    if (m_active)
    {
      RCLCPP_WARN(LOGGER, "Cannot activate, controller is already active");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // Get state handles.
    for (const auto &type : m_state_interface_types)
    {
      if (!controller_interface::get_ordered_interfaces(state_interfaces_,
                                                        m_joint_names,
                                                        type,
                                                        (type == hardware_interface::HW_IF_POSITION)
                                                            ? m_joint_state_pos_handles
                                                            : m_joint_state_vel_handles))
      {
        RCLCPP_ERROR(LOGGER, "Expected %zu '%s' state interfaces, got %zu.",
                     m_joint_names.size(),
                     type.c_str(),
                     (type == hardware_interface::HW_IF_POSITION)
                         ? m_joint_state_pos_handles.size()
                         : m_joint_state_vel_handles.size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    if (synchronizeJointStates())
    {
      RCLCPP_INFO(LOGGER, "Synchronized joint states");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to synchronize joint states");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!updateKinematics())
    {
      RCLCPP_ERROR(LOGGER, "Failed to update kinematics");
      return controller_interface::CallbackReturn::ERROR;
    }

    m_active = true;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointToCartesianController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    if (!m_active)
    {
      RCLCPP_WARN(LOGGER, "Cannot deactivate, controller is not active");
      return controller_interface::CallbackReturn::SUCCESS;
    }
    else
    {
      m_joint_state_pos_handles.clear();
      m_joint_state_vel_handles.clear();
      this->release_interfaces();
      m_active = false;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointToCartesianController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    if (!m_active)
    {
      RCLCPP_WARN(LOGGER, "Cannot update, controller is not active");
      return controller_interface::return_type::ERROR;
    }

    if (!synchronizeJointStates())
    {
      RCLCPP_ERROR(LOGGER, "Failed to synchronize joint states");
      return controller_interface::return_type::ERROR;
    }

    if (!updateKinematics())
    {
      RCLCPP_ERROR(LOGGER, "Failed to update kinematics");
      return controller_interface::return_type::ERROR;
    }

    publishStateFeedback();

    return controller_interface::return_type::OK;
  }

  bool JointToCartesianController::synchronizeJointStates()
  {
    if (m_joint_state_pos_handles.size() != m_number_of_joints)
    {
      RCLCPP_ERROR(LOGGER, "Number of joint state position handles does not match number of joints");
      return false;
    }
    for (size_t i = 0; i < m_joint_state_pos_handles.size(); ++i)
    {
      if (m_joint_state_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
      {
        m_current_positions(i) = m_joint_state_pos_handles[i].get().get_value();
        m_last_positions(i) = m_current_positions(i);
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Joint state position handle %zu is not of type position", i);
        return false;
      }
    }

    for (size_t i = 0; i < m_joint_state_vel_handles.size(); ++i)
    {
      if (m_joint_state_vel_handles[i].get().get_interface_name() == hardware_interface::HW_IF_VELOCITY)
      {
        m_current_velocities(i) = m_joint_state_vel_handles[i].get().get_value();
        m_last_velocities(i) = m_current_velocities(i);
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Joint state velocity handle %zu is not of type velocity", i);
        return false;
      }
    }
    return true;
  }

  bool JointToCartesianController::updateKinematics()
  {
    if (m_fk_pos_solver->JntToCart(m_current_positions, m_end_effector_pose) < 0)
    {
      RCLCPP_ERROR(LOGGER, "Failed to compute forward pose kinematics");
      return false;
    }

    if (m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions, m_current_velocities), m_end_effector_vel) < 0)
    {
      RCLCPP_ERROR(LOGGER, "Failed to compute forward vel kinematics");
      return false;
    }

    return true;
  }

  void JointToCartesianController::publishStateFeedback()
  {
    rclcpp::Time now = get_node()->now();
    if (m_feedback_pose_publisher->trylock())
    {

      tf2::Stamped<KDL::Frame> stamped_pose = tf2::Stamped<KDL::Frame>(m_end_effector_pose, tf2::timeFromSec(now.seconds()), m_robot_base_link);

      m_feedback_pose_publisher->msg_ = tf2::toMsg(stamped_pose);

      m_feedback_pose_publisher->unlockAndPublish();
    }
    if (m_feedback_twist_publisher->trylock())
    {
      tf2::Stamped<KDL::Twist> stamped_twist = tf2::Stamped<KDL::Twist>(m_end_effector_vel.GetTwist(), tf2::timeFromSec(now.seconds()), m_robot_base_link);

      m_feedback_twist_publisher->msg_ = tf2::toMsg(stamped_twist);

      m_feedback_twist_publisher->unlockAndPublish();
    }
  }

} // namespace joint_to_cartesian_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joint_to_cartesian_controller::JointToCartesianController, controller_interface::ControllerInterface)
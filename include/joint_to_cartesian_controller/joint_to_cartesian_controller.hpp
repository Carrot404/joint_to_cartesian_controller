/*
 * @Description: joint to cartesian controller
 * @version: 1.0.0
 * @Author: Songjie Xiao
 * @Email: songjiexiao@zju.edu.cn
 * @Date: 2023-09-08 21:06:38
 * @LastEditTime: 2023-09-09 15:15:35
 */

#ifndef JOINT_TO_CARTESIAN_CONTROLLER_HPP_
#define JOINT_TO_CARTESIAN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "controller_interface/controller_interface.hpp"
#include <hardware_interface/loaned_state_interface.hpp>
#include "realtime_tools/realtime_publisher.h"

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "joint_to_cartesian_controller_parameters.hpp"

namespace joint_to_cartesian_controller
{
  class JointToCartesianController : public controller_interface::ControllerInterface
  {
  public:
    JointToCartesianController() = default;
    ~JointToCartesianController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_init() override;

  protected:
    bool synchronizeJointStates();

    bool updateKinematics();

    void publishStateFeedback();

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        m_joint_state_pos_handles;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        m_joint_state_vel_handles;

    // Parameters from ROS for JointToCartesianController
    std::shared_ptr<joint_to_cartesian_controller::ParamListener> m_param_listener;
    joint_to_cartesian_controller::Params m_params;
    // Dynamic parameters
    std::string m_tf_prefix;
    std::string m_robot_description;
    std::string m_end_effector_link;
    std::string m_robot_base_link;

    KDL::Chain m_robot_chain;
    size_t m_number_of_joints;
    std::vector<std::string> m_joint_names;
    std::vector<std::string> m_state_interface_types;

    // Internal buffers
    KDL::JntArray m_current_positions;
    KDL::JntArray m_current_velocities;
    KDL::JntArray m_last_positions;
    KDL::JntArray m_last_velocities;

    KDL::Frame m_end_effector_pose;
    KDL::FrameVel m_end_effector_vel;

    // Forward kinematics
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fk_pos_solver;
    std::shared_ptr<KDL::ChainFkSolverVel_recursive> m_fk_vel_solver;

    realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::PoseStamped> m_feedback_pose_publisher;
    realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::TwistStamped> m_feedback_twist_publisher;

    bool m_initialized = {false};
    bool m_configured = {false};
    bool m_active = {false};
  };
}

#endif // JOINT_TO_CARTESIAN_CONTROLLER_HPP_
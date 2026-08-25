#ifndef KDL_CARTESIAN_TO_JOINT_HPP
#define KDL_CARTESIAN_TO_JOINT_HPP

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_msgs/msg/hiper_twist.hpp"
#include "manipulator_msgs/msg/hiper_pose.hpp"
#include "manipulator_msgs/msg/hiper_joint_state.hpp"


#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

class KdlCartesianToJoint : public rclcpp::Node
{
public:
    KdlCartesianToJoint();
    ~KdlCartesianToJoint();

private:
    bool initKDL();
    void robotJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void jStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void twistCallback(const manipulator_msgs::msg::HiperTwist::SharedPtr msg);
    void poseCallback(const manipulator_msgs::msg::HiperPose::SharedPtr msg);

    // Variables de configuración
    double joint_velocity_limit_;
    std::vector<std::string> joint_names_;

    // Elementos KDL
    KDL::Chain chain_;
    KDL::JntArray q_current_;
    std::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;
    KDL::JntArray joint_planning_positions_;
    KDL::JntArray next_joint_planning_positions_;

    // Suscriptores y Publicadores
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_robot;
    rclcpp::Subscription<manipulator_msgs::msg::HiperTwist>::SharedPtr sub_twist_input_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_input_;
    rclcpp::Subscription<manipulator_msgs::msg::HiperPose>::SharedPtr sub_planing_pose_input_;
    rclcpp::Publisher<manipulator_msgs::msg::HiperJointState>::SharedPtr pub_cmd_;
};

#endif // KDL_CARTESIAN_TO_JOINT_HPP
#pragma once

#include <stdexcept>
#include <cmath>
#include <fstream>
#include <tuple> // Para std::tie
#include <string>
#include <utility> // Para std::pair
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/pose.hpp>


// ---------------------------------------------------------
// Declaración de funciones auxiliares matemáticas y de pose
// ---------------------------------------------------------

Eigen::Matrix4d ParsePoseMatrix(const std::string &key, const std::string &poses_yaml_path);
void AddPoseMatrix(const std::string &key, const Eigen::Matrix4d &pose, const std::string &poses_yaml_path);

tf2::Quaternion MuliplyQuaternions(const tf2::Quaternion &q1, const tf2::Quaternion &q2);

tf2::Quaternion InverseQuaternion(const tf2::Quaternion &q);

// El valor por defecto (m = 1) debe ir EXCLUSIVAMENTE en la declaración del .hpp
tf2::Quaternion rot2Quat(const Eigen::Matrix3d &R, int m = 1);

Eigen::Matrix4d PoseToMatrix(const geometry_msgs::msg::Pose& pose);

std::pair<tf2::Vector3, tf2::Quaternion> PoseInterpolation(
    const Eigen::Matrix4d &start_pose,
    const Eigen::Matrix4d &end_pose,
    double lambda);

std::pair<tf2::Vector3, tf2::Quaternion> ComputeNextCartesianPose(
    const Eigen::Matrix4d &pose_0,
    const Eigen::Matrix4d &pose_1,
    const Eigen::Matrix4d &pose_2,
    double tau,
    double T,
    double t);
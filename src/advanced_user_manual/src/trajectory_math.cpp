#include "advanced_user_manual/trajectory_math.hpp" 

Eigen::Matrix4d ParsePoseMatrix(const YAML::Node &root, const std::string &key)
{
    if (!root[key] || !root[key].IsSequence() || root[key].size() != 4)
    {
        throw std::runtime_error("YAML key '" + key + "' must be a 4x4 matrix sequence");
    }

    Eigen::Matrix4d pose;
    for (int row = 0; row < 4; ++row)
    {
        const YAML::Node row_node = root[key][row];
        if (!row_node.IsSequence() || row_node.size() != 4)
        {
            throw std::runtime_error("YAML key '" + key + "' must contain rows of length 4");
        }
        for (int col = 0; col < 4; ++col)
        {
            pose(row, col) = row_node[col].as<double>();
        }
    }

    return pose;
}

tf2::Quaternion MuliplyQuaternions(const tf2::Quaternion &q1, const tf2::Quaternion &q2)
{
    double x1 = q1.x(), y1 = q1.y(), z1 = q1.z(), w1 = q1.w();
    double x2 = q2.x(), y2 = q2.y(), z2 = q2.z(), w2 = q2.w();

    double x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y_result = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    double z_result = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    double w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

    return tf2::Quaternion(x_result, y_result, z_result, w_result);
}

tf2::Quaternion InverseQuaternion(const tf2::Quaternion &q)
{
    double x = q.x(), y = q.y(), z = q.z(), w = q.w();
    double norm_sq = x*x + y*y + z*z + w*w;

    if (norm_sq < 1e-12)
    {
        throw std::runtime_error("Cannot invert zero-norm quaternion");
    }

    return tf2::Quaternion(-x / norm_sq, -y / norm_sq, -z / norm_sq, w / norm_sq);
}

// Nota: Hemos eliminado el "int m = 1" aquí porque ya está definido en el .hpp
tf2::Quaternion rot2Quat(const Eigen::Matrix3d &R, int m)
{
    int M = (m >= 0) ? 1 : -1;
    double w = M * std::sqrt(R(0, 0) + R(1, 1) + R(2, 2) + 1.0) / 2.0;
    double x, y, z;

    if (std::abs(w) > 1e-3)
    {
        x = (R(2, 1) - R(1, 2)) / (4.0 * w);
        y = (R(0, 2) - R(2, 0)) / (4.0 * w);
        z = (R(1, 0) - R(0, 1)) / (4.0 * w);
    }
    else
    {
        w = 0.0;
        x = M * std::sqrt((R(0, 0) + 1.0) / 2.0);
        y = M * std::sqrt((R(1, 1) + 1.0) / 2.0);
        z = M * std::sqrt((R(2, 2) + 1.0) / 2.0);
    }

    return tf2::Quaternion(x, y, z, w);
}

std::pair<tf2::Vector3, tf2::Quaternion> PoseInterpolation(
    const Eigen::Matrix4d &start_pose,
    const Eigen::Matrix4d &end_pose,
    double lambda)
{
    // Cálculo de la posición
    Eigen::Vector3d p0 = start_pose.block<3,1>(0,3);
    Eigen::Vector3d p1 = end_pose.block<3,1>(0,3);
    Eigen::Vector3d p = p0 + lambda * (p1 - p0);
    tf2::Vector3 p_interp(p.x(), p.y(), p.z());

    // Cálculo de la orientación
    Eigen::Matrix3d R0 = start_pose.block<3,3>(0,0);
    Eigen::Matrix3d R1 = end_pose.block<3,3>(0,0);

    tf2::Quaternion qA = rot2Quat(R0);
    tf2::Quaternion qB = rot2Quat(R1);
    tf2::Quaternion qA_inv = InverseQuaternion(qA);
    tf2::Quaternion qC = MuliplyQuaternions(qA_inv, qB);

    double wc = qC.w();
    tf2::Vector3 vc(qC.x(), qC.y(), qC.z());

    double thetac = 2 * std::acos( wc );
    tf2::Vector3 nc = vc / ( std::sin( thetac/2 ) );
    double theta_lambda = thetac * lambda;

    double w_rot = std::cos( theta_lambda / 2 );
    tf2::Vector3 v_rot = nc * std::sin( theta_lambda / 2 );

    tf2::Quaternion q_rot(v_rot.x(), v_rot.y(), v_rot.z(), w_rot);
    tf2::Quaternion q_interp  = MuliplyQuaternions(qA, q_rot);

    return {p_interp, q_interp};
}

std::pair<tf2::Vector3, tf2::Quaternion> ComputeNextCartesianPose(
    const Eigen::Matrix4d &pose_0,
    const Eigen::Matrix4d &pose_1,
    const Eigen::Matrix4d &pose_2,
    double tau,
    double T,
    double t)
{
    tf2::Vector3 p_interp;
    tf2::Quaternion q_interp;
    
    if (t < -T || t > T){
        throw std::out_of_range("Parameter t is outside [-T, T]");
    } else {
        if (t <= -tau){
            double lambda = (t + T)/T;
            std::tie(p_interp, q_interp) = PoseInterpolation(pose_0, pose_1, lambda);
        } else if (t >= tau){
            double lambda = t/T;
            std::tie(p_interp, q_interp) = PoseInterpolation(pose_1, pose_2, lambda);
        } else {
            // calcular orientación
            Eigen::Matrix3d R0 = pose_0.block<3,3>(0,0);
            Eigen::Matrix3d R1 = pose_1.block<3,3>(0,0);
            Eigen::Matrix3d R2 = pose_2.block<3,3>(0,0);
            
            tf2::Quaternion q0 = rot2Quat(R0);
            tf2::Quaternion q1 = rot2Quat(R1);
            tf2::Quaternion q2 = rot2Quat(R2);

            tf2::Quaternion q0_inv = InverseQuaternion(q0);
            tf2::Quaternion q1_inv = InverseQuaternion(q1);
            
            tf2::Quaternion q01 = MuliplyQuaternions(q0_inv, q1);
            tf2::Quaternion q12 = MuliplyQuaternions(q1_inv, q2);

            double w01 = q01.w();
            tf2::Vector3 v01(q01.x(), q01.y(), q01.z());
            double w12 = q12.w();
            tf2::Vector3 v12(q12.x(), q12.y(), q12.z());

            double theta01 = 2 * std::acos( w01 );
            tf2::Vector3 n01 = v01 / ( std::sin( theta01/2 ) );
            double theta12 = 2 * std::acos( w12 );
            tf2::Vector3 n12 = v12 / ( std::sin( theta12/2 ) );
            
            double theta_k1 = ( -std::pow((tau - t), 2) / (4*tau*T) ) * theta01 ;
            double theta_k2 = ( std::pow((tau + t), 2) / (4*tau*T) ) * theta12;

            double wk1 = std::cos(theta_k1 / 2);
            tf2::Vector3 vk1 = n01 * std::sin(theta_k1 / 2);
            tf2::Quaternion qk1(vk1.x(), vk1.y(), vk1.z(), wk1);
            
            double wk2 = std::cos(theta_k2 / 2);
            tf2::Vector3 vk2 = n12 * std::sin(theta_k2 / 2);
            tf2::Quaternion qk2(vk2.x(), vk2.y(), vk2.z(), wk2);

            tf2::Quaternion q1k1 = MuliplyQuaternions(q1, qk1);
            q_interp = MuliplyQuaternions(q1k1, qk2);

            // calcular posición
            Eigen::Vector3d P0 = pose_0.block<3,1>(0,3);
            Eigen::Vector3d P1 = pose_1.block<3,1>(0,3);
            Eigen::Vector3d P2 = pose_2.block<3,1>(0,3);

            Eigen::Vector3d deltaP01 = P1 - P0;
            Eigen::Vector3d deltaP12 = P2 - P1;

            Eigen::Vector3d p = P1 - ( std::pow((tau - t), 2) / (4*tau*T) ) * deltaP01 + ( std::pow((tau + t), 2) / (4*tau*T) ) * deltaP12;
            p_interp = tf2::Vector3(p.x(), p.y(), p.z());
        }
    }
    return {p_interp, q_interp};
}
#include "robot.h"


DH_Robot::DH_Robot(const DH_Parameters dh_parameters[]) {
    std::memcpy(joint_parameters, dh_parameters, sizeof(DH_Parameters[JOINT_COUNT]));
}

void DH_Robot::update_joint_parameters(size_t joint_id, const DH_Parameters dh_parameters) {
    joint_parameters[joint_id] = dh_parameters;
}

void DH_Robot::update_joint_positions(const float angles_deg[]) {
    for (size_t i = 0; i < JOINT_COUNT; i++) {
        joint_parameters[i].theta_deg = angles_deg[i];
    }
}

Matrix4f DH_Robot::make_dh_matrix(const DH_Parameters p) {
    float cos_theta = cosf(RAD(p.theta_deg));
    float sin_theta = sinf(RAD(p.theta_deg));
    float cos_alpha = cosf(p.alpha_rad);
    float sin_alpha = sinf(p.alpha_rad);

    return Matrix4f{
        {cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, p.a * cos_theta},
        {sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, p.a * sin_theta},
        {        0,              sin_alpha,              cos_alpha,             p.d},
        {        0,                      0,                      0,               1},
    };

}

Vector3f DH_Robot::calculate_forward_kinematics() {
    auto ee_transformation_matrix = Matrix4f::eye();
    for (size_t i = 0; i < JOINT_COUNT; i++) {
        auto dh_m = make_dh_matrix(joint_parameters[i]);
        ee_transformation_matrix = matmul(ee_transformation_matrix, dh_m);
    }

    auto end_effector_hom = Vector4f::zeros_homogenious();
    end_effector_hom = ee_transformation_matrix * end_effector_hom;

    return Vector3f::from_homogenious(end_effector_hom);
}

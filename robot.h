#ifndef ROBOT_H
#define ROBOT_H

#include "geometry.h"

struct DH_Parameters {
    float theta_deg;
    float alpha_rad;
    float a;
    float d;
};

class DH_Robot {
public:
    static const size_t JOINT_COUNT = 6;

    DH_Robot(const DH_Parameters dh_parameters[]);

    Vector3f calculate_forward_kinematics();
    void update_joint_parameters(size_t joint_id, const DH_Parameters dh_parameters);
    void update_joint_positions(const float angles_deg[]);

private:
    Matrix4f make_dh_matrix(const DH_Parameters p);

    DH_Parameters joint_parameters[JOINT_COUNT];
};

#endif // ROBOT_H

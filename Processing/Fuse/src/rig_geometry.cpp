#include "rig_geometry.h"

#include <algorithm>
#include <cctype>
#include <cmath>

bool parseImuFrame(const std::string& name, Eigen::Vector3f& diagonal)
{
    std::string n = name;
    std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return std::tolower(c); });
    if      (n == "identity") diagonal = Eigen::Vector3f( 1,  1,  1);
    else if (n == "rx180")    diagonal = Eigen::Vector3f( 1, -1, -1);
    else if (n == "ry180")    diagonal = Eigen::Vector3f(-1,  1, -1);
    else if (n == "rz180")    diagonal = Eigen::Vector3f(-1, -1,  1);
    else return false;
    return true;
}

Eigen::Matrix4f rotationAboutAxis(RigAxis axis, float degrees_signed)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    const float rad = degrees_signed * static_cast<float>(M_PI) / 180.0f;

    Eigen::Matrix3f R;
    switch (axis)
    {
        case RigAxis::X: R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitX()).toRotationMatrix(); break;
        case RigAxis::Y: R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitY()).toRotationMatrix(); break;
        case RigAxis::Z: R = Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()).toRotationMatrix(); break;
        default:         R = Eigen::Matrix3f::Identity();
    }
    T.block<3, 3>(0, 0) = R;
    return T;
}

Eigen::Matrix4f rotationAboutPoint(RigAxis axis, float degrees_signed,
                                    const Eigen::Vector3f& pivot)
{
    Eigen::Matrix4f T = rotationAboutAxis(axis, degrees_signed);
    // Rotating about a point leaves that point fixed: t = p - R*p.
    T.block<3, 1>(0, 3) = pivot - T.block<3, 3>(0, 0) * pivot;
    return T;
}

Eigen::Matrix3f rotationBetweenVectors(const Eigen::Vector3f& from,
                                        const Eigen::Vector3f& to)
{
    const float from_norm = from.norm();
    const float to_norm = to.norm();
    if (from_norm < 1e-6f || to_norm < 1e-6f)
        return Eigen::Matrix3f::Identity();

    const Eigen::Vector3f a = from / from_norm;
    const Eigen::Vector3f b = to / to_norm;

    // Eigen's two-vector constructor already builds the minimal rotation,
    // and handles the near-parallel and near-antiparallel cases.
    return Eigen::Quaternionf::FromTwoVectors(a, b).toRotationMatrix();
}

Eigen::Matrix4f rigGuess(const RigGeometry& rig,
                          bool have_rotation, double rotation_deg,
                          bool have_tilt, double tilt_deg,
                          bool& has_guess)
{
    has_guess = have_rotation || have_tilt;
    if (!has_guess) return Eigen::Matrix4f::Identity();

    const Eigen::Matrix4f R_rot = have_rotation
        ? rotationAboutPoint(rig.rotation_axis,
                              rig.rotation_sign * static_cast<float>(rotation_deg),
                              rig.rotation_pivot)
        : Eigen::Matrix4f::Identity();
    const Eigen::Matrix4f R_tilt = have_tilt
        ? rotationAboutPoint(rig.tilt_axis,
                              rig.tilt_sign * static_cast<float>(tilt_deg),
                              rig.tilt_pivot)
        : Eigen::Matrix4f::Identity();

    return rig.compose_tilt_first ? (R_rot * R_tilt) : (R_tilt * R_rot);
}

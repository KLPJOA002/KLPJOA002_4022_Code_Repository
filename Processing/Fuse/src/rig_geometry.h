#pragma once

#include <Eigen/Dense>

// Which axis of the camera frame a rig motion turns about.
enum class RigAxis { None, X, Y, Z };

// The rig's calibration: which axes tilt and rotation happen about, which
// way round each one runs, and how the two compose. This is the whole of
// what the pipeline needs to turn a folder's "_r45_a-5" into a pose, and
// it is shared by scan discovery (which builds the initial guesses) and by
// the rotation-direction test (which rebuilds them with the rotation
// mirrored, to see which way the rig actually turned).
struct RigGeometry
{
    RigAxis tilt_axis = RigAxis::None;
    float   tilt_sign = 1.0f;

    // Where the tilt axis actually passes through, in the camera's own
    // frame, in metres. This matters more than it looks: the camera is
    // mounted *off* the tilt head's pivot, so tilting by angle a swings the
    // optical centre through an arc rather than spinning it in place. A
    // seed that rotates about the origin instead is wrong by |(I-R)p|,
    // which grows with sin(a) - negligible at 5 degrees, larger than ICP's
    // whole correspondence distance by 25 degrees. Left at zero this
    // reproduces the old origin-rotation behaviour; the tilt calibration
    // stage estimates it from the scans that do align.
    Eigen::Vector3f tilt_pivot = Eigen::Vector3f::Zero();

    RigAxis rotation_axis = RigAxis::None;
    float   rotation_sign = 1.0f;

    // Same idea for the turntable axis. Not auto-estimated yet - the tilt
    // sweep gives many small steps to fit against, whereas rotation gives
    // only a handful of large ones - but settable if you know it.
    Eigen::Vector3f rotation_pivot = Eigen::Vector3f::Zero();

    // true  -> world = R_rotation * R_tilt   (tilt happens in the frame
    //                                          that then gets rotated -
    //                                          a tilt head on a turntable)
    // false -> world = R_tilt * R_rotation   (rotation first, tilt axis
    //                                          fixed in the world)
    bool compose_tilt_first = true;

    bool seedsRotation() const { return rotation_axis != RigAxis::None; }
    bool seedsTilt()     const { return tilt_axis != RigAxis::None; }
};

// A pure rotation of `degrees_signed` about `axis`, through the origin.
Eigen::Matrix4f rotationAboutAxis(RigAxis axis, float degrees_signed);

// The same rotation, but about an axis passing through `pivot` rather than
// through the origin: Translate(p) * R * Translate(-p). With a zero pivot
// this is identical to rotationAboutAxis.
Eigen::Matrix4f rotationAboutPoint(RigAxis axis, float degrees_signed,
                                    const Eigen::Vector3f& pivot);

// Combines the tilt and rotation components into one initial guess. Either
// component may be absent, in which case it contributes identity; if both
// are absent, has_guess comes back false and the caller should fall back to
// feature-based coarse alignment.
// The minimal ("geodesic") rotation carrying `from` onto `to`: a rotation
// about their common perpendicular, with no spin about the vectors
// themselves. For two gravity readings this is exactly the tilt the hinge
// applied, because a horizontal hinge axis *is* perpendicular to gravity -
// so the minimal rotation is the physically right choice, not just the
// mathematically convenient one.
Eigen::Matrix3f rotationBetweenVectors(const Eigen::Vector3f& from,
                                        const Eigen::Vector3f& to);

Eigen::Matrix4f rigGuess(const RigGeometry& rig,
                          bool have_rotation, double rotation_deg,
                          bool have_tilt, double tilt_deg,
                          bool& has_guess);

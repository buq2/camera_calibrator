#pragma once

#include <optional>
#include <tuple>
#include "types.hh"

namespace calibrator {

/// Estimate plane which fulfills:
/// a*x + b*y + c*y = 1
/// This function can only calculate finite planes and works best with points
/// near origin.
Plane EstimatePlaneFinite(const Point3D& p1, const Point3D& p2,
                          const Point3D& p3);

/// \return Normal of the plane. Normalized.
Point3D PlaneNormal(const Plane& plane);

/// Return rotation matrix which rotates the plane such that new_normal
/// is new normal for the plane. Can be used for example to rotate points
/// on a plane such that the rotated points have z=constant.
Matrix3 RotationMatrixFromPlane(const Plane& plane,
                                const Point3D& new_normal = Point3D::UnitZ());

/// Project point to a plane using projection direction.
/// If no projection direction is given, plane normal is used.
Point3D ProjectToPlane(
    const Plane& plane, const Point3D& p,
    const std::optional<Point3D>& projection_direction = std::nullopt);

Matrix3 EstimateHomography(const Points2D& p1, const Points2D& p2);
Matrix3 EstimateHomography(const Points2D& p1, const Points3D& p2);
Matrix3 EstimateHomography(const Points3D& p1, const Points2D& p2);

// Homographies should be ones which transform x=HX where X is the world
// points and x are the image points.
Matrix3 EstimateKFromHomographies(const std::vector<Matrix3>& Hs);

// \parma[in] K_inv Inverse of calibration matrix
// \param[in] H Homography which transforms x=HX where X is the world
//  points and x are the image points.
std::tuple<Matrix3, Point3D> RecoverExtrinsics(const Matrix3& K_inv,
                                               const Matrix3& H);

/// Fix estimated rotation matrix which does not fill all requirements for one
Matrix3 FixRotationMatrix(const Matrix3& R);

}  // namespace calibrator

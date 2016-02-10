/**
 * @file /sophus_ros_conversions/src/lib/geometry.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <Eigen/Geometry>
#include "../../include/sophus_ros_conversions/eigen.hpp"
#include "../../include/sophus_ros_conversions/geometry.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sophus_ros_conversions {

/*****************************************************************************
** Implementation
*****************************************************************************/

void poseMsgToSophus(const geometry_msgs::Pose &pose, Sophus::SE3f &se3)
{
  Eigen::Quaternion<Sophus::SE3f::Scalar> orientation;
  Sophus::SE3f::Point translation;
  pointMsgToEigen(pose.position, translation);
  quaternionMsgToEigen(pose.orientation, orientation);
  se3 = Sophus::SE3f(orientation, translation);  // TODO faster way to set this than reconstructing
}

geometry_msgs::Pose sophusToPoseMsg(const Sophus::SE3f& s) {
  geometry_msgs::Pose pose;
  Eigen::Vector3f translation = s.translation();
  pose.position = eigenToPointMsg(translation);
  Eigen::Quaternionf quaternion = s.unit_quaternion();
  pose.orientation = eigenToQuaternionMsg(quaternion);
  return pose;
}

// Sophus uses SE3f::Point as the translation type in the constructors of SE3f types.
void vector3MsgToSophus(const geometry_msgs::Vector3 &v, Sophus::SE3f::Point &translation)
{
  translation << v.x, v.y, v.z;
}


void transformMsgToSophus(const geometry_msgs::Transform &transform, Sophus::SE3f &se3)
{
  Sophus::SE3f::Point translation;
  Eigen::Quaternion<Sophus::SE3f::Scalar> orientation;
  vector3MsgToSophus(transform.translation, translation);
  quaternionMsgToEigen(transform.rotation, orientation);
  se3 = Sophus::SE3f(orientation, translation);  // TODO faster way to set this than reconstructing
}

Sophus::SE3f transformMsgToSophus(const geometry_msgs::Transform &transform)
{
  Sophus::SE3f T;
  transformMsgToSophus(transform, T);
  return T;
}

geometry_msgs::Transform sophusToTransformMsg(const Sophus::SE3f& se3) {
  geometry_msgs::Transform msg;
  msg.translation.x = se3.translation().x();
  msg.translation.y = se3.translation().y();
  msg.translation.z = se3.translation().z();
  msg.rotation.x = se3.unit_quaternion().x();
  msg.rotation.y = se3.unit_quaternion().y();
  msg.rotation.z = se3.unit_quaternion().z();
  msg.rotation.w = se3.unit_quaternion().w();
  return msg;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace sophus_ros_conversions

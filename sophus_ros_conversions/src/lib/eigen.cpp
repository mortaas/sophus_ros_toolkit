/**
 * @file /sophus_ros_conversions/src/lib/eigen.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/sophus_ros_conversions/eigen.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sophus_ros_conversions {

/*****************************************************************************
** Ros Msgs -> Eigen Classes
*****************************************************************************/

void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3f &e)
{
  e(0) = m.x;
  e(1) = m.y;
  e(2) = m.z;
}

void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaternionf &e)
{
  e = Eigen::Quaternionf(m.w, m.x, m.y, m.z);
}

Eigen::Quaternionf quaternionMsgToEigen(const geometry_msgs::Quaternion &m)
{
  return Eigen::Quaternionf(m.w, m.x, m.y, m.z);
}

/*****************************************************************************
 ** Eigen Classes -> Ros Msgs
 *****************************************************************************/

geometry_msgs::Point eigenToPointMsg(Eigen::Vector3f &e) {
  geometry_msgs::Point p;
  p.x = e.x();
  p.y = e.y();
  p.z = e.z();
  return p;
}

geometry_msgs::Quaternion eigenToQuaternionMsg(Eigen::Quaternionf &e) {
  geometry_msgs::Quaternion q;
  q.w = e.w();
  q.x = e.x();
  q.y = e.y();
  q.z = e.z();
  return q;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace sophus_ros_conversions

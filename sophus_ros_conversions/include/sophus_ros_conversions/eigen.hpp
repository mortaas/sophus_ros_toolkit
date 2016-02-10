/**
 * @file /sophus_ros_conversions/include/sophus_ros_conversions/eigen.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef sophus_ros_conversions_EIGEN_HPP_
#define sophus_ros_conversions_EIGEN_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sophus_ros_conversions {

/*****************************************************************************
** Interfaces
*****************************************************************************/

void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3f &e);
void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaternionf &e);
Eigen::Quaternionf quaternionMsgToEigen(const geometry_msgs::Quaternion &m);

geometry_msgs::Point eigenToPointMsg(Eigen::Vector3f &e);
geometry_msgs::Quaternion eigenToQuaternionMsg(Eigen::Quaternionf &e);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace sophus_ros_conversions

#endif /* sophus_ros_conversions_EIGEN_HPP_ */

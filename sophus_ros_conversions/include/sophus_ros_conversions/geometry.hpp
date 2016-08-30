/**
 * @file /sophus_ros_conversions/include/sophus_ros_conversions/geometry.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef sophus_ros_conversions_GEOMETRY_HPP_
#define sophus_ros_conversions_GEOMETRY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <sophus/se3.hpp>
#include <tf/transform_listener.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sophus_ros_conversions {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * Converts a ros pose message to a sophus pose.
 *
 * @param p
 * @param s
 */
void poseMsgToSophus(const geometry_msgs::Pose &p, Sophus::SE3f &s);

/**
 * Converts a sophus pose to a ros pose message.
 * @param s
 * @return
 */
geometry_msgs::Pose sophusToPoseMsg(const Sophus::SE3f& s);

/**
 * Converts vector3f, usually used as part of ros transform messages into
 * the translation part that gets used in Sophus::SE3f types. Note that
 * sophus uses SE3f::Point as the translation type in the constructors of SE3f types.
 */
void vector3MsgToSophus(const geometry_msgs::Vector3 &v, Sophus::SE3f::Point &translation);

/**
 * Converts ros transform message to a homogenous transform in sophus.
 * @param transform
 * @param se3
 */
void transformMsgToSophus(const geometry_msgs::Transform &transform, Sophus::SE3f &se3);


/**
 * Converts tf::StampedTransform to a homogenous transform in sophus.
 * @param transform
 * @param se3
 */
template<typename T>
void stampedTransformToSophus( const tf::StampedTransform & transform, Sophus::SE3Group<T> & se3 )
{
	Eigen::Quaternion<T> q;
	Eigen::Matrix<T,3,1> t;
	
	q.x() = transform.getRotation().getX();
	q.y() = transform.getRotation().getY();
	q.z() = transform.getRotation().getZ();
	q.w() = transform.getRotation().getW();
	t.x() = transform.getOrigin().getX();
	t.y() = transform.getOrigin().getY();
	t.z() = transform.getOrigin().getZ();
	se3 = Sophus::SE3Group<T>(q,t);
}

/**
 * Alternate form of ros transform -> sophus conversion.
 *
 * @param transform
 * @return Sophus::SE3f
 */
Sophus::SE3f transformMsgToSophus(const geometry_msgs::Transform &transform);

/**
 * Convert sophus transform to ros transform.
 * @param se3
 * @return geometry_msgs::Transform
 */
geometry_msgs::Transform sophusToTransformMsg(const Sophus::SE3f& se3);


/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace sophus_ros_conversions

#endif /* sophus_ros_conversions_GEOMETRY_HPP_ */


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

int queue_size;
std::string base_stabilized_frame;
std::string base_frame;
tf::TransformBroadcaster* broadcaster;
tf::StampedTransform transform_;
tf::Quaternion temporaryQuaternion;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, temporaryQuaternion);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(temporaryQuaternion).getRPY(roll, pitch, yaw);
  temporaryQuaternion.setRPY(roll, pitch, 0.0);
  transform_.setRotation(temporaryQuaternion);

  transform_.stamp_ = imu_msg.header.stamp;

  broadcaster->sendTransform(transform_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mercury_imu_tf");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("base_stabilized_frame", base_stabilized_frame, std::string("base_stabilized"));
  pn.param("base_frame", base_frame, std::string("base_link"));
  pn.param("queue_size", queue_size, int(10));
  
  broadcaster = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = base_stabilized_frame;
  transform_.child_frame_id_ = base_frame;

  ros::Subscriber imu_subscriber = n.subscribe("imu_topic", queue_size, imuCallback);
  ros::spin();

  delete broadcaster;
  return 0;
}
#include "odometry.h"

Odometry::Odometry():
  x_(0.0),
  y_(0.0),
  theta_(0.0)
{
  odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
  odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}

void Odometry::update(float dt, float speed_motor_left, float speed_motor_right)
{
  // Calculate linear and angular velocities
  double v = (speed_motor_left + speed_motor_right) / 2.0;  // Average of left and right speed
  double w = (speed_motor_right - speed_motor_left) / wheelbase;  // Angular velocity

  // Update robot's position and orientation
  double dx = v * cos(theta_) * dt;
  double dy = v * sin(theta_) * dt;
  double dtheta = w * dt;
  x_ += dx;
  y_ += dy;
  theta_ += dtheta;

  //calculate robot's heading in quaternion angle
  //ROS has a function to calculate yaw in quaternion angle
  float q[4];
  euler_to_quat(0, 0, theta_, q);

  //robot's position in x,y, and z
  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0.0;

  //robot's heading in quaternion
  odom_msg_.pose.pose.orientation.x = (double) q[1];
  odom_msg_.pose.pose.orientation.y = (double) q[2];
  odom_msg_.pose.pose.orientation.z = (double) q[3];
  odom_msg_.pose.pose.orientation.w = (double) q[0];

  odom_msg_.pose.covariance[0] = 0.001;
  odom_msg_.pose.covariance[7] = 0.001;
  odom_msg_.pose.covariance[35] = 0.001;

  //linear speed from encoders
  odom_msg_.twist.twist.linear.x = v;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.linear.z = 0.0;

  //angular speed from encoders
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = w;

  odom_msg_.twist.covariance[0] = 0.0001;
  odom_msg_.twist.covariance[7] = 0.0001;
  odom_msg_.twist.covariance[35] = 0.0001;
}

nav_msgs__msg__Odometry Odometry::getData()
{
  return odom_msg_;
}

const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q)
{
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

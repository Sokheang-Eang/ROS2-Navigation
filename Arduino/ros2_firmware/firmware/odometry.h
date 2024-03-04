#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>

class Odometry
{
  public:
    Odometry();
    void update(float vel_dt, float speed_motor_left, float speed_motor_right);
    nav_msgs__msg__Odometry getData();

  private:
    const void euler_to_quat(float x, float y, float z, float* q);

    nav_msgs__msg__Odometry odom_msg_;
    float x_;
    float y_;
    float theta_;
};

#endif

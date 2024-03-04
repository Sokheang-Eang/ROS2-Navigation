#include "variable.h"
#include "odometry.h"
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3_stamped.h>

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();            \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

rcl_publisher_t odom_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

Odometry odometry;
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/*-------------------- PID Control Motion  ----------*/
PID pid_left_motor(&left_motor_speed, &max_speed, &speed_req_left, Kp, Ki, Kd, DIRECT);    //  Speed motor left
PID pid_right_motor(&right_motor_speed, &max_speed, &speed_req_right, Kp, Ki, Kd, DIRECT); //  Speed motor right

void setup()
{
  set_microros_transports();
  Serial.begin(GLOBLE_BOADRATE);
  Serial1.begin(GLOBLE_BOADRATE);
  Serial2.begin(GLOBLE_BOADRATE);
  Serial3.begin(GLOBLE_BOADRATE);
  /*-------------------------------- attachInterupt Pin ------------------------------*/
  pinMode(EncoderI_A, INPUT_PULLUP);
  pinMode(EncoderI_B, INPUT_PULLUP);
  pinMode(EncoderII_A, INPUT_PULLUP);
  pinMode(EncoderII_B, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  /*-------------------------------- attachInterupt Pin ------------------------------*/
  attachInterrupt(digitalPinToInterrupt(EncoderI_A), ReadEncoderI_B, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderII_A), ReadEncoderII_B, RISING);
  // setting PID parameters
  pid_left_motor.SetSampleTime(5);
  pid_right_motor.SetSampleTime(5);
  pid_left_motor.SetOutputLimits(-max_robot_speed, max_robot_speed);
  pid_right_motor.SetOutputLimits(-max_robot_speed, max_robot_speed);
  pid_left_motor.SetMode(AUTOMATIC);
  pid_right_motor.SetMode(AUTOMATIC);
}
void loop() {
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
        destroyEntities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    moveBase();
    publishData();
  }
}
void twistCallback(const void * msgin)
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  prev_cmd_time = millis();
}

bool createEntities()
{
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "astrobot_base_node", "", &support));
  // create odometry publisher
  RCCHECK(rclc_publisher_init_default(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "odom/unfiltered"
          ));
  // create twist command subscriber
  RCCHECK(rclc_subscription_init_default(
            &twist_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"
          ));
  // create timer for actuating the motors at 50 Hz (1000/20)
  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(
            &control_timer,
            &support,
            RCL_MS_TO_NS(control_timeout),
            controlCallback
          ));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
  RCCHECK(rclc_executor_add_subscription(
            &executor,
            &twist_subscriber,
            &twist_msg,
            &twistCallback,
            ON_NEW_DATA
          ));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // synchronize time with the agent
  syncTime();
  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&odom_publisher, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  return true;
}

void moveBase()
{
  // brake if there's no command received, or when it's only the first command sent
  if (((millis() - prev_cmd_time) >= 200))
  {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
  }
  speed_req_x = twist_msg.linear.x * 10;
  speed_req_z = twist_msg.angular.z * 10;
  speed_req_left = speed_req_x - speed_req_z * (wheelbase / 2);
  speed_req_right = speed_req_x + speed_req_z * (wheelbase / 2);

  // PID Motor Speed Control
  motor_left_control();
  motor_right_control();
  // Control Motor ***
  MotorDrive(pwm_left, pwm_left, pwm_right, pwm_right);
  unsigned long now = millis();
  if ((now - prev_odom_update) >= LOOPTIME) {
    float vel_dt = now - prev_odom_update;
    prev_odom_update = now;
    // Read Encoder ***
    RotaryEncoder_L = RotaryEncoder_L - Rotary_L;
    RotaryEncoder_R = RotaryEncoder_R - Rotary_R;
    interrupts();
    // Read IMU Sensor ***
    UART2_rxBuffer();
    // ros odom calculation ***
    LastEncoder_L = Rotary_L;
    LastEncoder_R = Rotary_R;
    // calculated motor speed ***
    get_motor_speed();
    odometry.update(
      vel_dt, left_motor_speed, right_motor_speed
    );
    // reset encoder
    RotaryEncoder_L = LastEncoder_L;
    RotaryEncoder_R = LastEncoder_R;
  }
}

void publishData()
{

  struct timespec time_stamp = getTime();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}
struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

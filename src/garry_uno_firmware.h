#define __STDC_FORMAT_MACROS 1

#include <inttypes.h>
#include <stdio.h>

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

/* function definitions */
void led_left_eye_cb(const std_msgs::Bool& msg);
void led_right_eye_cb(const std_msgs::Bool& msg);

void servo_upper_cb(const geometry_msgs::Twist& msg);
void servo_lower_cb(const geometry_msgs::Twist& msg);

void cmd_vel_cb(const geometry_msgs::Twist& msg);
void turn_right(const double speed);
void turn_left(const double speed);
void forward(const double speed);
void backward(const double speed);
void run_motors(const double speed);

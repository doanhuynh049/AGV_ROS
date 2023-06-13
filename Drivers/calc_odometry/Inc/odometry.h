/*
 * odometry.h
 *
 *  Created on: Nov 24, 2022
 *      Author: SF
 */

#ifndef CALC_ODOMETRY_INC_ODOMETRY_H_
#define CALC_ODOMETRY_INC_ODOMETRY_H_

#include <math.h>
#include <stdio.h>

#include <geometry_msgs/msg/twist.h>
#include <firmware_msgs/msg/mcu_pose.h>


#include "main.h"
#include "i3g4250d_reg.h"
#include "lsm303agr_reg.h"


typedef struct {
	float x, y, theta, dt, init_theta;
	float vel_r, vel_l;
	float desired_v, desired_w;
} odometry_ctx_t;

typedef struct {
	float dKp, dKi, dKd;
	float dErrorTerm;
	float dIntergral;
	float u;
} PID_CONTROL_t;

extern TIM_HandleTypeDef htim1; // LEFT PWM
extern TIM_HandleTypeDef htim2; // LEFT PWM
extern TIM_HandleTypeDef htim3; // RIGHT PWM
extern TIM_HandleTypeDef htim4; // LEFT PWM
extern TIM_HandleTypeDef htim5; // RIGHT PW

void calc_odometry(float *x, float *y, float *theta, float *yaw, float *pre_yaw, float *ini_yaw);

void odometry_init();

void update_yaw();

void odometry_pid_init(PID_CONTROL_t *ctx);

void update_step();

void update_wheels_vel(geometry_msgs__msg__Twist *cmdVel);

void set_vel_wheels();


float calc_theta(float yaw, float prev_yaw);

void calc_angular_vel(odometry_ctx_t *ctx, PID_CONTROL_t *ctx_pid);

void calc_wheels_speed(odometry_ctx_t *ctx, PID_CONTROL_t *ctx_pid);

void calc_yaw_pitch_roll_6dof(int16_t *raw_acceleration, int16_t *raw_magnetic, float *roll, float *pitch, float *yaw);

void calc_yaw_pitch_roll_9dof(int16_t *raw_acceleration, int16_t *magnetic, float *angular_rate_dps , float *roll, float *pitch, float *yaw);


#endif /* CALC_ODOMETRY_INC_ODOMETRY_H_ */

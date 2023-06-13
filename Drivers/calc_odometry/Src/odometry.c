/*
 * odometry.c
 *
 *  Created on: Nov 24, 2022
 *      Author: SF
 */
#include <odometry.h>

#define PI	3.1415927f
#define R	0.034f
#define L	0.185f
#define meters_pertick 0.000511963f

#define PWM_MAX  400
#define PWM_MIN  120

odometry_ctx_t dev_ctx_odometry;

float angular_rate_dps[3], delta_gyro[3], delta_acceleration[3];
int16_t data_raw_magnetic[3], data_raw_acceleration[3] = {0, 0, 16348},
			  data_raw_angular_rate[3] =  {0, 0, 0};

stmdev_ctx_t dev_ctx_gyro, dev_ctx_xl, dev_ctx_mg;

float prev_yaw = 0, init_yaw = 0;
float roll_9dof, pitch_9dof, yaw_9dof, global_yaw, local_yaw = 0;

int yaw_step = 0;

void odometry_init() {
	odometry_ctx_t *ctx = &dev_ctx_odometry;

	__HAL_TIM_SetCounter(&htim4, 32768);
	__HAL_TIM_SetCounter(&htim2, 32768);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	theta_init(ctx);

	ctx->x = 0;
	ctx->y = 0;
	ctx->theta = 0;
	ctx->dt = 0.01;

	ctx->vel_r = 0;
	ctx->vel_l = 0;
}

void theta_init(odometry_ctx_t *ctx) {
	  lsm303agr_init(&dev_ctx_xl, &dev_ctx_mg);
	  i3g4250d_init(&dev_ctx_gyro);

	  i3G4250D_calib_gyro(&dev_ctx_gyro, data_raw_angular_rate , delta_gyro);

	  lsm303agr_calib_xl(&dev_ctx_xl, data_raw_acceleration, delta_acceleration);

	  lsm303agr_read_mag(&dev_ctx_mg, data_raw_magnetic);

	  i3g4250d_read_norm_gyro(&dev_ctx_gyro, data_raw_angular_rate, delta_gyro, angular_rate_dps);

	  lsm303agr_read_calibrated_xl(&dev_ctx_xl, data_raw_acceleration, delta_acceleration);

	  calc_yaw_pitch_roll_9dof(
			  data_raw_acceleration, data_raw_magnetic, angular_rate_dps,
			  &roll_9dof, &pitch_9dof, &yaw_9dof
			  );

	  ctx->init_theta = yaw_9dof;
}

void update_yaw() {
	prev_yaw = yaw_9dof;

	lsm303agr_read_mag(&dev_ctx_mg, data_raw_magnetic);
	lsm303agr_read_calibrated_xl(&dev_ctx_xl, data_raw_acceleration, delta_acceleration);
	i3g4250d_read_norm_gyro(&dev_ctx_gyro, data_raw_angular_rate, delta_gyro, angular_rate_dps);

	calc_yaw_pitch_roll_9dof(
	  data_raw_acceleration, data_raw_magnetic, angular_rate_dps,
	  &roll_9dof, &pitch_9dof, &yaw_9dof
	);

}

void get_odom_msg(firmware_msgs__msg__McuPose *msg) {
	msg->x = dev_ctx_odometry.x;
	msg->y = dev_ctx_odometry.y;
	msg->theta = dev_ctx_odometry.theta;
	msg->v_left = (int) dev_ctx_odometry.vel_l;
	msg->v_right = (int) dev_ctx_odometry.vel_r;
}

void odometry_pid_init(PID_CONTROL_t *ctx) {
	ctx->dKp = 0; // 21 16.6
	ctx->dKi = 0; // 5   4
	ctx->dKd = 0; // 2   1.6
	ctx->dErrorTerm = 0;
	ctx->dIntergral = 0;
	ctx->u = 0;
}

void update_step()
{
	odometry_ctx_t *ctx  = &dev_ctx_odometry;

	float delta_ticks_left = ((int32_t) TIM4->CNT - 32768);
	float delta_ticks_right = ((int32_t) TIM2->CNT - 32768);

	  __HAL_TIM_SetCounter(&htim4, 32768);
	  __HAL_TIM_SetCounter(&htim2, 32768);

	float wl = delta_ticks_left / 1980.0 * 2.0 * PI;
	float wr = delta_ticks_right / 1980.0 * 2.0 * PI;
	float mTheta = calc_theta(yaw_9dof, ctx->init_theta);

	float vl = wl*R;
	float vr = wr*R;
	float v = (vl + vr) / 2.0;

	ctx->x += v*cosf(mTheta);
	ctx->y += v*sinf(mTheta);
	ctx->theta = mTheta;
}

void update_vel_wheels(geometry_msgs__msg__Twist *cmdVel) {

	float pwmLeftReq =  ((2 * cmdVel->linear.x - cmdVel->angular.z * L) / (2* R));
	float pwmRightReq = ((2 * cmdVel->linear.x + cmdVel->angular.z * L) / (2* R));

	float absPwmLeftReq = abs(pwmLeftReq);
	float absPwmRightReq = abs(pwmRightReq);


//	if (absPwmLeftReq < PWM_MIN ) {
//		pwmLeftReq = 0;
//	}
//
//	if (absPwmRightReq < PWM_MIN ) {
//		pwmRightReq = 0;
//	}

	if (pwmRightReq  > 0) { // Right wheel forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else  { // Right wheel reverse
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	if (pwmLeftReq > 0) { // Left wheel forward
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	} else { // Left wheel reverse
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	pwmLeftReq   = (absPwmLeftReq > PWM_MAX) ? PWM_MAX : absPwmLeftReq;
	pwmRightReq  = (absPwmRightReq > PWM_MAX)? PWM_MAX : absPwmRightReq;

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmLeftReq);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmRightReq);

	dev_ctx_odometry.vel_l = pwmLeftReq;
	dev_ctx_odometry.vel_r = pwmRightReq;
}


void set_vel_wheels() {

	int pwmLeftReq = (int) dev_ctx_odometry.vel_l ;
	int pwmRightReq = (int) dev_ctx_odometry.vel_r ;

	if (pwmRightReq  > 0) { // Right wheel forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else  { // Right wheel reverse
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	if (pwmLeftReq > 0) { // Left wheel forward
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	} else { // Left wheel reverse
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(pwmLeftReq));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(pwmRightReq));

}

void run_step(odometry_ctx_t *ctx, PID_CONTROL_t *ctx_pid) {

	calc_angular_vel(ctx, ctx_pid);

	calc_wheels_speed(ctx, ctx_pid);

	if (ctx->vel_r > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int) ctx->vel_r);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int) abs(ctx->vel_r));
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	if (ctx->vel_l > 0) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int) abs(ctx->vel_l) );
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int) abs(ctx->vel_l) );
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	}

};

void pid_compute(odometry_ctx_t *ctx, PID_CONTROL_t *PID_Ctrl, float g_dPIDError)
{
	float dP = 0, dI = 0, dD = 0;
	float TS = ctx->dt;

	dP = PID_Ctrl->dKp  * g_dPIDError;
	PID_Ctrl->dIntergral += g_dPIDError;
	dI = PID_Ctrl->dKi  * TS / 2 * PID_Ctrl->dIntergral;
	dD = PID_Ctrl->dKd  * (g_dPIDError - PID_Ctrl->dErrorTerm) / TS;
	PID_Ctrl->dErrorTerm = g_dPIDError;
	PID_Ctrl->u = dP + dI + dD;

}

void calc_angular_vel(odometry_ctx_t *ctx, PID_CONTROL_t *ctx_pid) {
//	float delta_x =  ctx->x_goal - ctx->x;
//	float delta_y = ctx->y_goal - ctx->y;
//
//	if (fabs(delta_x) < 0.03 && fabs(delta_y) < 0.03) {
//		ctx->vel_r = 0;
//		ctx->desired_v = 0;
//		ctx->vel_l = 0;
//		ctx->delay_counts++;
//
//		if (ctx->delay_counts < 100 ) {
//			return;
//		}
//
//		if (ctx->next_goal == 6 ) {
//			return;
//		}
//
//
//		ctx->x_goal = ctx->goals[ctx->next_goal];
//		ctx->y_goal = ctx->goals[ctx->next_goal + 1];
//		ctx->next_goal = ctx->next_goal + 2;
//		ctx->delay_counts = 0;
//		ctx->desired_v = 1.5;
//		ctx_pid->dErrorTerm = 0;
//		ctx_pid->dIntergral = 0;
//		ctx_pid->u = 0;
//
//		return;
//	}
//
//	float delta_theta = atan2(delta_y, delta_x) - ctx->theta;
//	delta_theta = atan2(sin(delta_theta), cos(delta_theta));
//
//	pid_compute(ctx, ctx_pid, delta_theta);
}

void calc_wheels_speed(odometry_ctx_t *ctx, PID_CONTROL_t *ctx_pid) {
	float v = ctx->desired_v;
	float w = ctx_pid->u;

	ctx->vel_l = (2 * v + w * L) / (2 * R);
	ctx->vel_r = (2 * v - w * L) / (2 * R);
}

float calc_theta(float yaw, float prev_yaw) {
	float rate = (yaw - prev_yaw);

	if (rate >= PI) {
		rate = rate - 2*PI;
	} else if (rate <= -PI ) {
		rate = rate + 2*PI;
	}

	return rate;
}

void calc_yaw_pitch_roll_6dof(int16_t *data_raw_acceleration, int16_t *data_raw_magnetic, float *roll, float *pitch, float *yaw) {
	 // Phi
    *roll = atan2(data_raw_acceleration[1], data_raw_acceleration[2]);
    // Theta
    *pitch = atan(-data_raw_acceleration[0] / (data_raw_acceleration[1] * sin(*roll) + data_raw_acceleration[2] * cos(*roll)));

    float magn_fy_fs = data_raw_magnetic[2]  * sin(*roll) - data_raw_magnetic[1]*cos(*roll);
    float magn_fx_fs = data_raw_magnetic[0] * cos(*pitch) + data_raw_magnetic[1] * sin(*pitch) * sin(*roll) +  data_raw_magnetic[2] * sin(*pitch) * cos(*roll);

    *yaw = atan2(magn_fy_fs, magn_fx_fs);

    if (*yaw < 0) {
    	*yaw = *yaw + 2*PI;
    }
}

void calc_yaw_pitch_roll_9dof(int16_t *data_raw_acceleration, int16_t *data_raw_magnetic, float *angular_rate_dps , float *roll, float *pitch, float *yaw) {
	 // Phi
	float phi = atan2(data_raw_acceleration[1], data_raw_acceleration[2]);
    // Theta
	float theta = atan(-data_raw_acceleration[0] / (data_raw_acceleration[1] * sin(*roll) + data_raw_acceleration[2] * cos(*roll)));

	float magn_fy_fs = data_raw_magnetic[2]  * sin(*roll) - data_raw_magnetic[1]*cos(*roll);
    float magn_fx_fs = data_raw_magnetic[0] * cos(*pitch) + data_raw_magnetic[1] * sin(*pitch) * sin(*roll) +  data_raw_magnetic[2] * sin(*pitch) * cos(*roll);

    float psi = atan2(magn_fy_fs, magn_fx_fs);

    float phi_dot = angular_rate_dps[0] + angular_rate_dps[1]*sin(phi) * tan(theta) + angular_rate_dps[2]* cos(phi) * tan(theta);
    float theta_dot = angular_rate_dps[1] * cos(phi) - angular_rate_dps[2] * sin(phi);
    float psi_dot = angular_rate_dps[1]*sin(phi) / cos(theta) + angular_rate_dps[2] * cos(phi) / cos(theta);

    *roll = phi + phi_dot*0.01;
    *pitch  = theta + theta_dot*0.01;
    *yaw = psi + psi_dot*0.01;

    if (*yaw < 0) {
    	*yaw = *yaw + 2*PI;
    }

}

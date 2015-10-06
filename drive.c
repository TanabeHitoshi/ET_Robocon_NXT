/*
 * drive.c
 *
 *  Created on: 2014/11/09
 *      Author: ｈｔ
 */
#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */
#include "ini.h"
#include "drive.h"

int turn = 0, speed = 0; // 旋回速度、走行速度

//*****************************************************************************
// 関数名 : line_follow
// 引数 : speed, turn 走行速度、旋回速度
// 引数 : gyro_sensor　ジャイロセンサー値
// 返り値 : 無し
// 概要 : ライントレース
//*****************************************************************************
void line_follow(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
}
//*****************************************************************************
// 関数名 : line_follow2
// 引数 : Black (黒のセンサ値)
// 引数 : white (白のセンサ値)
// 返り値 : 無し
// 概要 : バランサーを使用しないライントレース
//*****************************************************************************
void line_follow2(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;
	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn2 = 50;
	if (turn2 < -50) turn2  = -50;
	pwm_L = speed - turn2;
	pwm_R = speed + turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
	//xsprintf(tx_buf,"lf2:turn2=%d,%d\n",turn2,ecrobot_get_light_sensor(NXT_PORT_S3));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
}

void line_follow3(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;

	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn2 = 50;
	if (turn2 < -50) turn2  = -50;
	pwm_L = speed + turn2;
	pwm_R = speed - turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
}
//*****************************************************************************
// 関数名 : turn_left_follow
// 引数 : speed, turn 走行速度、旋回速度
// 引数 : gyro_sensor　ジャイロセンサー値
// 返り値 : 無し
// 概要 : ライントレース
//*****************************************************************************
void turn_left_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L-50, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R+40, 1); /* 右モータPWM出力セット(-100～100) */
}
//*****************************************************************************
// 関数名 : turn_right_follow
// 引数 : speed, turn 走行速度、旋回速度
// 引数 : gyro_sensor　ジャイロセンサー値
// 返り値 : 無し
// 概要 : ライントレース
//*****************************************************************************
void turn_right_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L+10, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R-10, 1); /* 右モータPWM出力セット(-100～100) */
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* 比例制御 */
	/* PWM出力飽和処理 */
	if (pwm > PWM_ABS_MAX)	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX) {
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}


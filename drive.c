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

/*
 * isPositione.c
 *
 *  Created on: 2015/03/13
 *      Author: ｈｔ
 */

#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */
#include "ini.h"
#include "isLineSensor.h"
#include "isPosition.h"

//*****************************************************************************
// 関数名 : tripmeter
// 引数 : なし	NXT_PORT_C(左), NXT_PORT_B(右)
// 返り値 : 走行距離（ｍｍ）
// 概要 : エンコーダーで走行距離を測定する
//*****************************************************************************
int tripmeter(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = (nxt_motor_get_count(NXT_PORT_C) + nxt_motor_get_count(NXT_PORT_B)); // エンコーダ左右合計
	return (((s / 360) * circumference) + (circumference * (s % 360) / 360)) / 2;
}

int tripmeter_left(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = nxt_motor_get_count(NXT_PORT_C); // エンコーダ左
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}

int tripmeter_right(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = nxt_motor_get_count(NXT_PORT_B); // エンコーダ右
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}
//*****************************************************************************
// 関数名 : check_position
// 引数 :なし
// 返り値 : グローバル変数x, yを更新
// 概要 : 自己位置座標推定
//*****************************************************************************
void check_position(void)
{
	int l_arc, r_arc;
	float l_distance, r_distance, distance;
	float theta;

	now.l_enc = nxt_motor_get_count(NXT_PORT_C);
	now.r_enc = nxt_motor_get_count(NXT_PORT_B);

	l_arc = now.l_enc - prev.l_enc;
	r_arc = now.r_enc - prev.r_enc;

	l_distance = 254.0 * l_arc / 360.0;
	r_distance = 254.0 * r_arc / 360.0;
	distance = (l_distance + r_distance) / 2.0;

	theta = (l_arc - r_arc) / 140.0;

	now.x = prev.x + (distance * cos(prev.dir + theta / 2.0));
	now.y = prev.y + (distance * sin(prev.dir + theta / 2.0));
	now.dir = theta + prev.dir;

	prev = now;
}

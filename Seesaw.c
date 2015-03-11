/*
 * Seesaw.c
 *
 *  Created on: 2015/03/11
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
#include "Seesaw.h"

//*****************************************************************************
// 関数名 : check_Seesaw
// 引数 :なし
// 返り値 : 連続数
// 概要 : ジャイロセンサの値が一定以上連続した場合カウントする
//*****************************************************************************

int check_Seesaw(int gyro_sensor)
{
	static int prev_gyro, diff_gyro, cnt_gyro = 0;
	diff_gyro = gyro_sensor - prev_gyro;
	if (diff_gyro > 15) {cnt_gyro++;} else {cnt_gyro = 0;}
	prev_gyro = gyro_sensor;
	return cnt_gyro;
}

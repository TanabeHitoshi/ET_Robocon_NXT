/*
 * isLineSensor.c
 *
 *  Created on: 2014/11/06
 *      Author: ｈｔ
 */

#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */
#include "isLineSensor.h"

float kp = KP;
int course = 0; /* 走行するコース IN or OUT */
//*****************************************************************************
// 関数名 : math_limit
// 引数 : 値、下限値、上限値
// 戻り値 : 下限値＜値<上限値
// 概要 :
//*****************************************************************************

float math_limit(float val, float min, float max)
{
	if(val < min) {
		return min;
	} else if(val > max) {
		return max;
	}

	return val;
}


//*****************************************************************************
// 関数名 : pid_sample
// 引数 : sensor_val (センサー値), target_val(目標値)
// 返り値 : 操作量
// 概要 :PID制御サンプル（下記のところからのコピー）
// ETロボコンではじめるシステム制御（4）
// 滑らかで安定したライントレースを実現する」
// http://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html
//*****************************************************************************

float pid_control(int sensor_val, int target_val)
{
	float p =0, i=0, d=0;

	static signed int diff[2] = {0, 0};
	static float integral = 0.0;

	diff[0] = diff[1];
	diff[1] = sensor_val - target_val;	//偏差を取得
	integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

	p = kp * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;
	//xsprintf(tx_buf,"pid:s=%d,t=%d,pid=%d\n",sensor_val,target_val,(int)(p+i+d));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
	if (course == OUT) 	return  -math_limit(p + i + d, -100.0, 100.0);
	else return  math_limit(p + i + d, -100.0, 100.0);
}


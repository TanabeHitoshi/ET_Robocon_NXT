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
#include "drive.h"
#include "isLineSensor.h"

float kp = KP;
float ki = KI;
float kd = KD;

int course = 0; /* 走行するコース IN or OUT */
int black = 508, white = 664; // 白の値，黒のセンサ値
int black2 = 559, white2 = 746; // 傾倒時の白黒のセンサ値

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
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / DELTA_T;
	//xsprintf(tx_buf,"pid:s=%d,t=%d,pid=%d\n",sensor_val,target_val,(int)(p+i+d));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
	if (course == L_course) 	return  -math_limit(p + i + d, -100.0, 100.0);
	else return  -math_limit(p + i + d, -100.0, 100.0);
}
//*****************************************************************************
// 関数名 : calibration
// 引数 : *black (黒、最小値)，*white（白、最大値）
// 返り値 : 無し
// 概要 : 光センサの手動キャリブレーション
//        黒白の順でタッチする。
//*****************************************************************************
void calibration(int *black,int *white,int angle)
{
	while(1) {
		tail_control(angle);

		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			ecrobot_sound_tone(440, 170, 100);
			*black = ecrobot_get_light_sensor(NXT_PORT_S3);
			display_clear(0);		/* 画面表示 */
			display_goto_xy(0, 1);
			display_string("BLACK:");
			display_int(*black, 4);
			break;
		}//if
		systick_wait_ms(100); /* 10msecウェイト */
	}//while
	display_update();
	while(ecrobot_get_touch_sensor(NXT_PORT_S4));
	ecrobot_sound_tone(880, 170, 100);

	while(1) {
		tail_control(angle);
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			ecrobot_sound_tone(880, 170, 100);
			*white = ecrobot_get_light_sensor(NXT_PORT_S3);
			//display_clear(0);		/* 画面表示 */
			display_goto_xy(0, 2);
			display_string("WHITE:");
			display_int(*white, 4);
			break;
		}//if
		systick_wait_ms(100); /* 10msecウェイト */
	}//while

	//display_clear(0);		/* 画面表示 */
	display_goto_xy(0,4);
	display_string("TH:");
	display_int(TH(*black,*white), 3);
	display_update();
	while(ecrobot_get_touch_sensor(NXT_PORT_S4));
	ecrobot_sound_tone(440, 170, 100);
}


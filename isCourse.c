/*
 * isCourse.c
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
#include "isCourse.h"




//*****************************************************************************
// 関数名 : check_marker
// 引数 : black (黒のセンサ値)
// 引数 : white (白のセンサ値)
// 返り値 : マーカー発見ならば１そうでないなら０
// 概要 : バランサーを使用しないライントレースTH(black, white)
//*****************************************************************************
int check_marker(int turn)


{
	static int r=0,l=0;
	if(turn < -20) {
		r++;
	} else {
		if (turn > 0) {
			r = 0;
		}
	}
	if(turn > 20) {
		l++;
	} else {
		if (turn < 0) {
			l = 0;
		}
	}

	//xsprintf(tx_buf,"%4d, %4d\n",turn,tripmeter());
	//ecrobot_send_bt(tx_buf,1, 12);

	if (r >= 5) {
		r = 0;
		ecrobot_sound_tone(440*4 , 10, 100);
		return 1; //右エッジ走行のときはマーカースタート
	}
	if (l >= 5) {
		l = 0;
		//ecrobot_sound_tone(440*2 , 10, 100);
		return -1;
	}
	return 0;
}
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
// 関数名 : check_course
// 引数 : 距離
// 走行距離で現在のコース状況をナビ
// 返り値 : 配列の添え字
//*****************************************************************************
int check_course(int distance)
{
	int i, found = CMAX - 1;

	for (i=0; i<CMAX; i++) {
		if (course == IN) { //in
			if (distance < in_course[i].distance) {
				found = i-1;
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		} else if (course == OUT){ //out
			if (distance < out_course[i].distance) {
				found = i-1;
				//printf("out\n");
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		} else {
			if (distance < test_course[i].distance) {
				found = i-1;
				//printf("out\n");
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		}

	}

	return found;
}


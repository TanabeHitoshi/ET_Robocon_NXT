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
#include "Seesaw.h"
#include "drive.h"
#include "isLineSensor.h"
#include "isCourse.h"
#include "isPosition.h"
#include "bluetooth.h"
#include "LookUpGate.h"

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
	if (diff_gyro > 30) {cnt_gyro++;} else {cnt_gyro = 0;}
	prev_gyro = gyro_sensor;
	return cnt_gyro;
}

//*****************************************************************************
// 関数名 : seesaw
// 引数 :なし
// 返り値 : 1(処理終了)/0(処理中)
// 概要 : シーソー難所の動作
//*****************************************************************************
int seesaw(void)
{
	static unsigned int SS_pattern = 10;

	switch(SS_pattern){
		case 10: /*** シーソー：200mm速度を落としてアプローチ ***/
			//xsprintf(tx_buf,"%4d, %3d\n",tripmeter(),gyro_sensor);
			//ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			//if (tripmeter() - measure0 > 180 ) {
			//	counter = 0;
			//	measure0 = tripmeter();
			//	pattern = 31;
			//}
			if (check_Seesaw(gyro_sensor)>2) {
				ecrobot_sound_tone(440*3, 100, 100);
				measure0 = tripmeter();
				counter = 0;
				SS_pattern = 20;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20: /*** シーソー：150mmスピードをあげてシーソーに乗り上げる ***/
			if (tripmeter() - measure0 > 30 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 30;
			}
			speed = 70;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 30: /*** シーソー：250mmゆっくり登る ***/
			if (tripmeter() - measure0 > 450 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 40;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 40: /*** シーソー：70mm下りはブレーキで ***/
			if (tripmeter() - measure0 > 70 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 50;
			}
			speed = -50;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 50: /*** シーソー：200mm姿勢が安定するまでゆっくり進む ***/
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
			}
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			break;
	}
	if(SS_pattern == 50)
		return 1;	/* シーソー動作終了 */
	else
		return 0;	/* シーソー動作中 */
}

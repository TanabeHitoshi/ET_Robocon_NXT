/*
 * LookUpGate.c
 *
 *  Created on: 2015/03/14
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

//*****************************************************************************
// 関数名 : lookupgate
// 引数 : 無し
// 返り値 : 1(処理終了)/0(処理中)
// 概要 : ルックアップゲート処理
//*****************************************************************************
int lookupgate( void )
{
	static unsigned int LUT_pattern = 10;
	signed int fangle = 50; // 傾倒時のオフセット角

	pattern = pattern * 100 +LUT_pattern;

	switch(LUT_pattern){
		case 10:
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(440, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				LUT_pattern =20;
			}
			break;

		case 20: /*** ルックアップゲート：後ろに傾倒 ***/
			speed = 30;
			//tail_control(TAIL_ANGLE_STAND_UP);  // 尻尾を出す
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4　で強制的に後ろに傾ける
			//ecrobot_sound_tone(660, 1, 100);
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* 右モータPWM出力セット(-100～100) */

			if (counter > 1) { // 50ms * 5 この状態を保つ
				counter = 0;
				LUT_pattern = 30;
			}
			break;

		case 30: /*** ルックアップゲート：尻尾をゆっくり角度を減らして、走行体を寝かす ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				LUT_pattern =40;
				measure0 = tripmeter();
			}
			break;

		case 40:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (counter > 30 ) {
				counter = 0;
				LUT_pattern =50;
				measure0 = tripmeter();
			}
			break;

		case 50:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 30;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
				LUT_pattern =60;
			}
			break;

		case 60:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = -30;
			nxt_motor_set_speed(NXT_PORT_C, speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B, speed, 1); /* 右モータPWM出力セット(-100～100) */

		//			line_follow3(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 < -350 ) {
				counter = 0;
				LUT_pattern =70;
				measure0 = tripmeter();
			}
			break;

		case 70:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 30;

			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				LUT_pattern =80;
				measure0 = tripmeter();
			}
			break;

		case 80:/***  ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle + counter/2 );
			if (counter > 64 ) {
					counter = 0;
					LUT_pattern =90;
					measure0 = tripmeter();
				}
			break;

		case 90:/***  ***/
			speed = 20;
			line_follow2(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 420 ) {
				counter = 0;
				LUT_pattern =10;
				measure0 = tripmeter();
				}
			break;
	}

	pattern = pattern /100;

	if(LUT_pattern == 90)
		return 1;	/* ルックアップゲート動作終了 */
	else
		return 0;	/* ルックアップゲート動作中 */

}

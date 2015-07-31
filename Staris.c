/*
 * Staris.c
 *
 *  Created on: 2015/03/15
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
// 関数名 : staris
// 引数 : 無し
// 返り値 : 1(処理終了)/0(処理中)
// 概要 : 階段の動作
//*****************************************************************************
int stairs( void )
{

	switch(ST_pattern){
		case 10:/* 段差直前のトレース 速度を落として走行 */
			speed = 20;//20
			kp = 0.5;
//			tail_control(TAIL_ANGLE_STAND_UP - 25);
			if( check_Seesaw(gyro_sensor) > 2 ){
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 20;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20:/* 段差検知、速度を上げて登る */
			speed = 200;
			line_follow(speed, 0, gyro_sensor + 10);
			if (tripmeter() - measure0 > 10 ){
				counter = 0;
				ST_pattern = 30;
			}
			break;

		case 30:/* 少し後ろにバックして体制を整える */
			speed = -10;
			line_follow(speed, 5, gyro_sensor);
			if (counter > 50 ){
				counter = 0;
//				ST_pattern = 40;
				ST_pattern = 90;
			}
			break;

		case 40:/* ラインの検出しながら前へ */
			kp = 0.3;
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				ST_pattern = 50;
			}			break;

		case 50:/* ラインの検出しながら前へ */
			kp = 0.5;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (counter > 200 ){
				counter = 0;
				ST_pattern = 60;
			}
			break;
		case 60:/* ラインの検出 */
			kp = 0.3;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black ){
				counter = 0;
				ST_pattern = 70;
				measure0 = tripmeter_right();
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 70:/* ターン */
			turn_left_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > (502 - 20) ){
				counter = 0;
				ST_pattern = 80;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 80:/* ラインの検出 */
			turn_right_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > 60 ){
				counter = 0;
				ST_pattern = 90;
			}
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black){
				counter = 0;
				ST_pattern = 90;
			}
			break;

		case 90:/* 段差の検知 */
			kp = KP;
			speed = 40;
			if( check_Seesaw(gyro_sensor)>2 ){
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 100;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 100:/* 少し前に進む */
			speed = 20;
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				ST_pattern = 110;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 110:/* マーカーを見つける*/
			speed = 40;
			line_follow(speed, turn, gyro_sensor);
			if (check_marker(turn)>0) {
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 120;

			}
			break;

		case 120:/* テールを下ろす*/
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				ST_pattern =130;
			}
			break;

		case 130: /*** 後ろに傾倒 ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // 尻尾を出す
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4　で強制的に後ろに傾ける
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* 右モータPWM出力セット(-100～100) */

			if (counter > 1) { // 50ms * 5 この状態を保つ
				counter = 0;
				ST_pattern =140;
			}
			break;

		case 140: /*** 尻尾をゆっくり角度を減らして、走行体を寝かす ***/
			speed = 0;
			line_follow3(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				ST_pattern =150;
				measure0 = tripmeter();
			}
			break;

		case 150:/*** ゲートまで進む ***/
			speed = 20;
			line_follow3(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 350 ) {
				counter = 0;
				measure0 = tripmeter();
				}
			break;
	}

	if(ST_pattern == 150)
		return 1;	/* 階段動作終了 */
	else
		return 0;	/* 階段動作中 */

}

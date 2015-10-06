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
	static int sa;

	switch(ST_pattern){
		case 5:
			speed = 0;
			tail_control(TAIL_ANGLE_STAND_UP - 10);
			nxt_motor_set_speed(NXT_PORT_B,100,1);
			nxt_motor_set_speed(NXT_PORT_C,100,1);
			if(counter < 100){
				ST_pattern = 15;
				counter = 0;
			}
			break;
		case 10:/* 段差直前のトレース 速度を落として走行 */
			speed = 20;//20
			kp = 0.7;
			kd = 0.03;
			tail_control(TAIL_ANGLE_STAND_UP - 40);
//			if( check_Seesaw(gyro_sensor) > 1 && counter > 50 && (max_position() - tripmeter()) > 10){
			if( counter > 50 && (max_position() - tripmeter()) > 10){
				counter = 0;
				measure_P = tripmeter();
				measure_L = tripmeter_left();
				measure_R = tripmeter_right();
				ST_pattern = 20;
			}
			measure0 = tripmeter();
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20:/* 段差検知、速度を上げて登る */
			ecrobot_sound_tone(880, 170, 100);
			speed = 130;
			line_follow(speed, 0, gyro_sensor + 0);
			if (tripmeter() - measure0 > 10 ){
				counter = 0;
				ST_pattern = 30;
			}
			break;

		case 30:/* 少し後ろにバックして体制を整える */
			speed = -5;
			line_follow(speed, 5, gyro_sensor);
			if (counter > 40 ){
				counter = 0;
				ST_pattern = 40;
//				ST_pattern = 90;
			}
			break;

		case 40:/* ラインの検出しながら前へ */
			speed = 5;
			line_follow(speed, -10, gyro_sensor);
			if(light_sensor < TH(black, white)){
				counter = 0;
				ST_pattern = 50;
			}
			break;
		case 50:/* ラインの検出しながら前へ */
			if ((tripmeter() - measure_P) > 400 ){
				speed = -20;		/* しばらく体制を整える */
			}else{
				speed = 20;	/* 前に行き過ぎていれば下がる*/
			}
//			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (counter > 50 ){
				counter = 0;
				ST_pattern = 60;
				measure0 = tripmeter_right();
				sa = 500 - ((tripmeter_right() - measure_R) - (tripmeter_left() - measure_L));
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 60:/* ターン */
			turn_left_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > sa ){
				counter = 0;
				ST_pattern = 70;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 70:/* ラインの検出 */
			speed = 10;
			line_follow(speed, 10, gyro_sensor);
			if(light_sensor < TH(black, white)){
				counter = 0;
				ST_pattern = 75;
			}
			break;

		case 75:/* ラインの検出 */
			speed = 10;
			if( tripmeter() - measure_P > 600){
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 80;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 80:/* 段差の検知 */
			kp = KP;
			speed = 60;
			if( check_Seesaw(gyro_sensor)>3 || tripmeter() - measure_P > 700){
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
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			if (check_marker(turn)>1) {
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 120;
				ecrobot_sound_tone(880, 170, 100);
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
				pattern = 160;
				measure0 = tripmeter();
				}
			break;
		default:
			break;
	}

	if(ST_pattern == 100)
		return 1;	/* 階段動作終了 */
	else
		return 0;	/* 階段動作中 */

}

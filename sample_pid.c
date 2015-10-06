/** ******************************************************************************
 **	ファイル名 : sample_pid.c
 **
 **	概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cサンプルプログラム
 **
 ** 注記 : (sample_c4のPID制御version)
 ********************************************************************************/

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
#include "Stairs.h"
#include "Garage.h"
#include "log.h"

/* OSEK declarations */
DeclareTask(Task1);

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET 600 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) 600*/
#define LIGHT_WHITE 500 /* 白色の光センサ値 500*/
#define LIGHT_BLACK 700 /* 黒色の光センサ値 700*/
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] 30*/
/* sample_c3マクロ */
#define DEVICE_NAME       "ET005"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */
#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */


//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{

	signed int tail_angle = 0, tail_cnt = 0; // TAIL用カウンタ 
	unsigned int loop_start, loop_time; // ループ時間計測用 
//	signed int fangle = 40; // 傾倒時のオフセット角

	ecrobot_device_initialize();
	pattern = 0; //初期状態

	xsprintf(tx_buf,"**** HELLO! ****\n");
	ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;

	while(1) {
		if (tripmeter() > 19800 && pattern < 100) pattern = 100;/*19.9mでストップ*/

		loop_start = systick_get_ms();// whileループの周期を4msに保つためにループ開始時刻を記憶。
		//4ms毎にジャイロセンサ、光センサ、回転方向、
		gyro_sensor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
		turn = pid_control(ecrobot_get_light_sensor(NXT_PORT_S3), TH(black, white));
		light_sensor = ecrobot_get_light_sensor(NXT_PORT_S3);

		//xsprintf(tx_buf,"%4d\n",gyro);
		//ecrobot_send_bt(tx_buf,0,6);

		switch (pattern) {

		case 0:  /*** キャリブレーション ***/
			ecrobot_sound_tone(880, 170, 100);
			 /* 手動キャリブレーション */
			calibration(&white2, &black2, TAIL_ANGLE_STAND_UP - fangle);
			calibration(&white, &black, TAIL_ANGLE_STAND_UP);
			systick_wait_ms(500);
			pattern = 1;
			break;

		case 1: /*** スタート待ち ***/
			tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

			if (turn < 5 && turn > -5) ecrobot_sound_tone(1500, 1, 30);
			
			course = remote_start();
			if (course  > 0 && sonar == 255) {
				course = L_course;
				ecrobot_sound_tone(880, 170, 100);
				pattern = 5;
				break;
			} else if (course > 0 ) {
				course = R_course;
				ecrobot_sound_tone(220, 170, 100);
				pattern = 5;
				break; /* リモートスタート */
			}

			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar < 150) {
				course = R_course;
				ecrobot_sound_tone(880, 170, 100);
				systick_wait_ms(10); /* 10msecウェイト */
				pattern = 5;
				break; /* タッチセンサが押された */
			}
			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar == 255) {
				course = L_course;
				ecrobot_sound_tone(440, 170, 100);
				systick_wait_ms(10); /* 10msecウェイト */
				pattern = 5;
				break; /* タッチセンサが押された */
			}

			break;

		case 5: /*** 走行準備 ***/
			tail_cnt = 0;
			tail_angle = TAIL_ANGLE_STAND_UP;
			balance_init();						/* 倒立振子制御初期化 */
			nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
			nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */
			/* ソナー接続確認 */

			systick_init();						/* 走行時間リセット */
			pattern = 10;
//			pattern = 300;
			break;

		case 10: /* スタートダッシュ */
			// 走行開始直後は速度を抑えて走行しながら、ゆっくりテールを上げる
			tail_cnt++;
			if( tail_cnt > 10 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
			}
			speed = 60; // 通常はこの速度
			kp = 0.2;
			ki = KI;
			kd = KD;
			line_follow(speed, turn, gyro_sensor);

			tail_control(tail_angle); // バランス走行用角度に制御
			if (tripmeter()> 500 ) {
				counter = 0;
				pattern = 11;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;
		case 11: /*** 通常走行中 ***/
			// 走行開始直後は速度を抑えて走行しながら、ゆっくりテールを上げる
			tail_cnt++;
			if( tail_cnt > 20 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
			}
			speed = 126 - tail_angle; // 通常はこの速度
			kp = 0.2;
//			kp = 0.8;
			ki = KI;
			kd = KD;
			line_follow(speed, turn, gyro_sensor);
			tail_control(tail_angle); // バランス走行用角度に制御
			if (tripmeter()> 4134 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
			{
				pattern = 12;//29.21　Ｌ
				counter = 0;
				ecrobot_sound_tone(880, 170, 100);
				break;
			}
			if((tripmeter() > 4032) && ( course == R_course) ){
				counter = 0;
				pattern = 21;//27.90　Ｒ
				ecrobot_sound_tone(440 * 3, 200, 100);
			}

#if 0
			if(tripmeter() > 600){
				counter = 0;
				pattern =30; //40
				ecrobot_sound_tone(440 * 3, 200, 100);
			}
#endif
			break;

		case 12://L 第2区間
			speed = 100; // 通常はこの速度
			kp = 0.4;
			ki = KI;
			kd = KD;
			if (tripmeter()> 4710 && course == L_course)
			{
				pattern = 13;
				counter = 0;
				ecrobot_sound_tone(880, 170, 100);
				break;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 13://L 第3区間
				speed = 65; // 通常はこの速度
				kp = 0.6;
				ki = KI;
				kd = KD;
				if (tripmeter()> 5390 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
				{
					pattern = 14;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 14://L 第4区間
				speed = 100; // 通常はこの速度
				kp = 0.2;
				ki = KI;
				kd = KD;
				if (tripmeter()> 6534 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
				{
					pattern = 15;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 15://L 第5区間
				speed = 58; // 通常はこの速度
				kp = 0.7;
				ki = KI;
				kd = KD;
				if (tripmeter()> 7774 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
				{
					pattern = 16;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;
		case 16://L 第6区間
				speed = 120; // 通常はこの速度
				kp = 0.5;
				ki = KI;
				kd = KD;
				if (tripmeter()> 8937 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
				{
					pattern = 30;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 21://R 第2区間
					speed = 65; // 通常はこの速度
					kp = 0.6;
					ki = KI;
					kd = KD;
					if (tripmeter()> 4869 && course == R_course)
					{
						pattern = 22;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 22://R 第3区間
					speed = 110; // 通常はこの速度
					kp = 0.2;
					ki = KI;
					kd = KD;
					if (tripmeter()> 6094 && course == R_course)
					{
						pattern = 23;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 23://R 第4区間
					speed = 58; // 通常はこの速度
					kp = 0.7;
					ki = KI;
					kd = KD;
					if (tripmeter()> 7333 && course == R_course)
					{
						pattern = 24;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 24://R 第5区間
					speed = 127; // 通常はこの速度
					kp = 0.2;
					ki = KI;
					kd = KD;
					if (tripmeter()> 8419 && course == R_course)
					{
						pattern = 25;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;
		case 25://R 第6区間
					speed = 55; // 通常はこの速度
					kp = KP;
					ki = KI;
					kd = KD;
					if (tripmeter()> 9819 )
					{
						pattern = 40;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 30:/* ルックアップゲート */
			line_follow(speed, turn, gyro_sensor);
			speed = 40;
			if(sonar < 30 ){	//30
				measure_P = measure0 = tripmeter();
				counter = 0;
				pattern = 31;
			}
			break;
		case 31:/* ルックアップゲート */
			line_follow(speed, turn, gyro_sensor);
			speed = 20;
			if(sonar < 20 ){	//30
				measure_P = measure0 = tripmeter();
				speed = 0;
				counter = 0;
				pattern = 32;
			}
			break;
		case 32:	/* ルックアップゲート */
			if(lookupgate()){
					pattern = 33;
					counter = 0;
				}
			break;
		case 33:/***  ***/
			speed = 25;
			line_follow2(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			if (tripmeter() - measure_P > 1230) {//ゲートからガレージまでの距離
				counter = 0;
				pattern =34;
				measure0 = tripmeter();
				}
			break;
		case 34:/***  ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,0,1);
			nxt_motor_set_speed(NXT_PORT_C,0,1);
			break;

		case 40:/* 段差直前のトレース速度を落として走行 */
			if(stairs()){
				pattern = 41;
				measure0 = tripmeter();
				ecrobot_sound_tone(880, 170, 100);
			}
			break;
		case 41:
			speed = 30;
			line_follow(speed, turn, gyro_sensor);
			if (tripmeter() - measure_P > 1000){
				counter = 0;
				pattern = 42;
			}
			break;

		case 42: /* ガレージイン */
			garage();
			pattern =43;
			break;
		case 43:/***  ***/
			speed = 30;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			if (tripmeter() - measure_P > 1723 - 40 ) {//段差からガレージまでの距離
				counter = 0;
				pattern =44;
				measure0 = tripmeter();
				}
			break;
		case 44:/***  ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,0,1);
			nxt_motor_set_speed(NXT_PORT_C,0,1);
			break;

		case 2000: /* 倒立で停止 */
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			break;
		case 300:/*テスト走行*/
			// 走行開始直後は速度を抑えて走行しながら、ゆっくりテールを上げる
			tail_cnt++;
			if( tail_cnt > 10 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
			}
			speed = 40; // 通常はこの速度
			kp = 0.5;
			ki = KI;
			kd = KD;
			line_follow(speed, turn, gyro_sensor);

			tail_control(tail_angle); // バランス走行用角度に制御
			if (tripmeter()> 400 ) {
				counter = 0;
				pattern = 11;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;
		case 301: /*** 通常走行中 ***/
			// 走行開始直後は速度を抑えて走行しながら、ゆっくりテールを上げる
			tail_cnt++;
			if( tail_cnt > 20 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
			}
			speed = 40 - tail_angle; // 通常はこの速度
			kp = KP;
			ki = KI;
			kd = KD;
			line_follow(speed, turn, gyro_sensor);
			tail_control(tail_angle); // バランス走行用角度に制御
			if (tripmeter()> 100000 && course == L_course) // ルックアップゲート検知ならpattern=20 16000
			{
				pattern = 12;//29.21　Ｌ
				counter = 0;
				ecrobot_sound_tone(880, 170, 100);
				break;
			}
			if((tripmeter() > 7000) && ( course == R_course) ){
				counter = 0;
				pattern = 40;//27.90　Ｒ
				ecrobot_sound_tone(440 * 3, 200, 100);
			}
			break;

		default:
			break;

		}//switch
 		
		/* できるだけ4msecでループ */
		loop_time = systick_get_ms() - loop_start;
		if (loop_time<=4) {
			systick_wait_ms(4 - loop_time);
		}//
		//systick_wait_ms(4);
	}//while

}//TASK MAR_course

///////////////////////////////////////////////////////////////////////////////

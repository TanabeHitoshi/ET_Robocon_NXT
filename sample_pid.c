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

/* OSEK declarations */

DeclareTask(Task1);
DeclareTask(TaskLogger);

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET 600 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) 600*/
#define LIGHT_WHITE 500 /* 白色の光センサ値 500*/
#define LIGHT_BLACK 700 /* 黒色の光センサ値 700*/
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] 30*/
/* sample_c3マクロ */
/* sample_c4マクロ */
#define DEVICE_NAME       "ET315"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */
#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */

/* log用構造体 */
typedef struct {
	int distance;	//走行距離
	int pattern;	//走行パターン
	int speed;		//走行速度
	int turn;		//ターン速度
	char state;		//コース状態
	int x;			//Ｘ座標
	int y;			//Ｙ座標
	int dir;		//方向
} DATA_LOG_t;


DATA_LOG_t data_log;

/* PID制御用 */

static int pattern = 0; /* ロボットの状態 */
static int sonar = 255; //超音波センサ(無探知は255)
static int light_sensor; //超音波センサ(無探知は255)
static int navi = 0, navi0 = 0;

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{

//	signed int black = 508, white = 664; // 白の値，黒のセンサ値
//	signed int black2 = 559, white2 = 746; // 傾倒時の白黒のセンサ値
	signed int tail_angle = 0, tail_cnt = 0; // TAIL用カウンタ 
	unsigned int loop_start, loop_time; // ループ時間計測用 


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
				course = OUT;
				ecrobot_sound_tone(880, 170, 100);
				pattern = 10;
				break;
			} else if (course > 0 ) {
				course = IN;
				ecrobot_sound_tone(220, 170, 100);
				pattern = 10;
				break; /* リモートスタート */
			}

			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar < 150) {
				course = IN;
				ecrobot_sound_tone(880, 170, 100);
				systick_wait_ms(10); /* 10msecウェイト */
				pattern = 10;
				break; /* タッチセンサが押された */
			}
			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar == 255) {
				course = OUT;
				ecrobot_sound_tone(440, 170, 100);
				systick_wait_ms(10); /* 10msecウェイト */
				pattern = 10;
				break; /* タッチセンサが押された */
			}

			break;

		case 10: /*** 走行準備 ***/
			tail_cnt = 0;
			tail_angle = TAIL_ANGLE_STAND_UP;
			balance_init();						/* 倒立振子制御初期化 */
			nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
			nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */
			/* ソナー接続確認 */

			systick_init();						/* 走行時間リセット */
			pattern = 11;
			break;

		case 11: /*** 通常走行中 ***/
			// 走行開始直後は速度を抑えて走行しながら、ゆっくりテールを上げる
			tail_cnt++;
			if( tail_cnt > 10 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
				speed = 10; // テールアップ中はこの速度
			} else {
				speed = 100; // 通常はこの速度
			}
			tail_control(tail_angle); // バランス走行用角度に制御

			if (sonar < 30 && tripmeter()> 11000 && course == OUT) // ルックアップゲート検知ならpattern=20 16000
			{
				pattern = 20;
				counter = 0;
				break;
			}
#if 1
			if((tripmeter() > 10200) && ( course == IN) ){
//			if((tripmeter() > 400) && ( course == IN) ){
			counter = 0;
					pattern = 40;
					ecrobot_sound_tone(440 * 3, 200, 100);
			}
#endif

			//if(tripmeter()==17000) ecrobot_sound_tone(440 * 3, 200, 100); // 17000
#if 0
			if (tripmeter() > 10000) { //シーソー直前のマーカーに近づいたら、マーカー検知 20000
				if (check_marker(turn)>0) {
					counter = 0;
					measure0 = tripmeter();
					pattern = 30;
				}
			}
#endif
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20:	/* ルックアップゲート */
			if(lookupgate()){
				pattern = 100;
			}
			break;

		case 30: /* シーソー　*/
			if(seesaw()){
				pattern = 100;
			}
			break;

		case 40:/* 段差直前のトレース速度を落として走行 */
			if(stairs()){
				pattern = 100;
			}
			break;

		case 100: /* ガレージイン */
			garage();
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

}//TASK MAIN

///////////////////////////////////////////////////////////////////////////////

//*****************************************************************************
// タスク名 : TaskLogger
// 概要 : ロガータスク 50msごとに起動
// 超音波センサー、カウンターインクリメント、xprintfフラッシュ
//*****************************************************************************
/* Task1 executed every 50msec */
TASK(TaskLogger)
{
	counter++;

	//if (course == OUT || course == TEST || course == 0) {
		sonar = ecrobot_get_sonar_sensor(NXT_PORT_S2);
	//	if ( sonar < 0 ) sonar = 255;
	//} else {
	//	sonar = 255;
	//}

	if(pattern > 10 && pattern <= 101) { //走行中のとき
		navi = check_course(tripmeter());
		if(navi0 != navi) ecrobot_sound_tone(440*4, 200, 100);

		data_log.distance = tripmeter();
		data_log.pattern = pattern;
		data_log.speed = speed;
		data_log.turn = turn;

		check_position();

		data_log.x = (int)now.x;
		data_log.y = (int)now.y;
		data_log.dir = (int)now.dir;

		if (course == IN) {
			data_log.state = in_course[navi].state;
		} else if(course == OUT){
			data_log.state = out_course[navi].state;
		} else {
			data_log.state = test_course[navi].state;
		}
		navi0 = navi;

		xsprintf(tx_buf,"%6d,%4d,%4d,%4d,%c,%6d,%6d,%4d,%4d \n",
				data_log.distance,
				data_log.pattern,
				data_log.speed,
				data_log.turn,
				data_log.state,
				data_log.x,
				data_log.y,
				data_log.dir,
				sonar
		);


		ecrobot_send_bt(tx_buf,0,strlen(tx_buf));

	}
	/* send Sensor/Motors/NXT internal status to the host.
	 * NXT GamePad in the host PC accumulates all logging data
	 * and later you can save the logging data into a CSV file
	 */
	//ecrobot_bt_data_logger(i++, j--);

	/* display Sensors/Motors/NXT internal status */
	//ecrobot_status_monitor("Data Logging");
	//display_clear(0);		/* 画面表示 */
	display_goto_xy(0,3);
	display_string("pattern:");
	display_int(pattern, 3);
	display_goto_xy(0,5);
	display_string("trip:");
	display_int(tripmeter(), 6);
	display_goto_xy(0,6);
	display_string("left :");
	display_int(nxt_motor_get_count(NXT_PORT_C), 6);
	display_goto_xy(0,7);
	display_string("right:");
	display_int(nxt_motor_get_count(NXT_PORT_B), 6);

	display_update();

	//ecrobot_sound_tone(880, 2, 100);
	TerminateTask();
}//TaskLogger

///////////////////////////////////////////////////////////////////////////////



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

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
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
#define TAIL_ANGLE_STAND_UP 105 /* 完全停止時の角度[度] 108*/
#define TAIL_ANGLE_DRIVE      0 /* バランス走行時の角度[度] 3*/
/* sample_c4マクロ */
#define DEVICE_NAME       "ET315"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */
#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */
/* PID制御マクロ */

#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1



/* 関数プロトタイプ宣言 */
static int remote_start(void);
static int strlen(const char *s);

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];
char tx_buf[128];



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
//static signed int diff[2];
//static float integral;

static int pattern = 0; /* ロボットの状態 */

static int sonar = 255; //超音波センサ(無探知は255)
static int turn = 0, speed = 0; // 旋回速度、走行速度
static int light_sensor; //超音波センサ(無探知は255)
static int navi = 0, navi0 = 0;
static int gyro_sensor = 255; // ジャイロセンサの値

static unsigned int counter=0; /* TaskLoggerにより 50ms ごとにカウントアップ */
static unsigned int cnt_ms=0; /* OSEKフック関数により 1ms？ ごとにカウントアップ */


//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値 : なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;

	cnt_ms++;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{

	signed int black = 508, white = 664; // 白の値，黒のセンサ値 
	signed int black2 = 559, white2 = 746; // 傾倒時の白黒のセンサ値 
	signed int tail_angle = 0, tail_cnt = 0; // TAIL用カウンタ 
	signed int fangle = 32; // 傾倒時のオフセット角
	unsigned int loop_start, loop_time; // ループ時間計測用 
	int measure0 = 0; // 距離測定用（計測スタート地点）


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

		case 20:
			speed = 0;

			line_follow(speed, turn, gyro_sensor);

			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				pattern =201;
			}
			break;

		case 201: /*** ルックアップゲート：後ろに傾倒 ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // 尻尾を出す
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4　で強制的に後ろに傾ける
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* 右モータPWM出力セット(-100～100) */

			if (counter > 1) { // 50ms * 5 この状態を保つ
				counter = 0;
				pattern =21;
			}
			break;

		case 21: /*** ルックアップゲート：尻尾をゆっくり角度を減らして、走行体を寝かす ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				pattern =22;
				measure0 = tripmeter();
			}
			break;

		case 22:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (counter > 30 ) {
				counter = 0;
				pattern =23;
				measure0 = tripmeter();
			}
			break;

		case 23:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 30;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 300 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern =24;
			}
			break;

		case 24:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = -20;
			nxt_motor_set_speed(NXT_PORT_C, speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B, speed, 1); /* 右モータPWM出力セット(-100～100) */

//			line_follow3(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 < -250 ) {
				counter = 0;
				pattern =25;
				measure0 = tripmeter();
			}
			break;
		case 25:/*** ルックアップゲート：傾倒状態で30*50ミリ秒ライントレース(この状態でゲート通過) ***/
			speed = 30;

			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 300 ) {
				counter = 0;
				pattern =26;
				measure0 = tripmeter();
			}
			break;

		case 26:/***  ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle + counter/2 );
			if (counter > 64 ) {
					counter = 0;
					pattern =27;
					measure0 = tripmeter();
				}
			break;

		case 27:/***  ***/
			speed = 20;
			line_follow2(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 420 ) {
				counter = 0;
				pattern =101;
				measure0 = tripmeter();
				}
			break;

		case 30: /*** シーソー：200mm速度を落としてアプローチ ***/
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
				pattern = 31;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 31: /*** シーソー：150mmスピードをあげてシーソーに乗り上げる ***/
			if (tripmeter() - measure0 > 30 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 32;
			}
			speed = 70;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 32: /*** シーソー：250mmゆっくり登る ***/
			if (tripmeter() - measure0 > 450 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 33;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 33: /*** シーソー：70mm下りはブレーキで ***/
			if (tripmeter() - measure0 > 70 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 34;
			}
			speed = -50;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 34: /*** シーソー：200mm姿勢が安定するまでゆっくり進む ***/
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 100;
			}
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 40:/* 段差直前のトレース速度を落として走行 */
			speed = 20;
//			tail_control(TAIL_ANGLE_STAND_UP - 25);
			if( check_Seesaw(gyro_sensor) > 2 ){
				counter = 0;
				measure0 = tripmeter();
				pattern = 41;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 41:/* 段差検知、速度を上げて登る */
			speed = 100;
			line_follow(speed, 0, gyro_sensor + 10);
			if (tripmeter() - measure0 > 30 ){
				counter = 0;
				pattern = 42;
			}
			break;

		case 42:/* 少し後ろにバックして体制を整える */
			speed = -10;
			line_follow(speed, 5, gyro_sensor);
			if (counter > 50 ){
				counter = 0;
//				pattern = 43;
				pattern = 46;
			}
			break;

		case 43:/* ラインの検出しながら前へ */
			kp = 0.3;
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				pattern = 431;
			}			break;

		case 431:/* ラインの検出しながら前へ */
			kp = 0.5;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (counter > 200 ){
				counter = 0;
				pattern = 44;
			}
			break;
		case 44:/* ラインの検出 */
			kp = 0.3;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black ){
				counter = 0;
				pattern = 45;
				measure0 = tripmeter_right();
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 45:/* ターン */
			turn_left_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > (502 - 20) ){
				counter = 0;
				pattern = 455;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 455:/* ラインの検出 */
			turn_right_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > 60 ){
				counter = 0;
				pattern = 46;
			}
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black){
				counter = 0;
				pattern = 46;
			}
			break;

		case 46:/* 段差の検知 */
			kp = KP;
			speed = 40;
			if( check_Seesaw(gyro_sensor)>2 ){
				counter = 0;
				measure0 = tripmeter();
				pattern = 47;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 47:/* 少し前に進む */
			speed = 20;
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				pattern = 49;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 48:/* マーカーを見つける*/
			speed = 40;
			line_follow(speed, turn, gyro_sensor);
			if (check_marker(turn)>0) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 49;

			}
			break;

		case 49:/* テールを下ろす*/
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				pattern =50;
			}
			break;

		case 50: /*** 後ろに傾倒 ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // 尻尾を出す
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4　で強制的に後ろに傾ける
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* 左モータPWM出力セット(-100～100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* 右モータPWM出力セット(-100～100) */

			if (counter > 1) { // 50ms * 5 この状態を保つ
				counter = 0;
				pattern =51;
			}
			break;

		case 51: /*** 尻尾をゆっくり角度を減らして、走行体を寝かす ***/
			speed = 0;
			line_follow3(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				pattern =52;
				measure0 = tripmeter();
			}
			break;

		case 52:/*** ゲートまで進む ***/
			speed = 20;
			line_follow3(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 350 ) {
				counter = 0;
				pattern =101;
				measure0 = tripmeter();
				}
			break;

		case 100: /*** 走行停止：尻尾を出しながら、一瞬加速 ***/
			counter = 0;
			speed = 0;
			turn = 0;
			pattern = 101;
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,100,1);
			nxt_motor_set_speed(NXT_PORT_C,100,1);
			xsprintf(tx_buf,"%4d ---101101--\n",tripmeter());
			ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			systick_wait_ms(200);
			break;

		case 101: /*** 走行停止：左右モーター停止で静止状態に ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			speed = 0;
			//ecrobot_sound_tone(440*3, 1000, 100);
			while (1) {
				tail_control(TAIL_ANGLE_STAND_UP - 20);
				nxt_motor_set_speed(NXT_PORT_B,0,1);
				nxt_motor_set_speed(NXT_PORT_C,0,1);
				//line_follow2(speed, black2, white2);
				//ecrobot_sound_tone(440*3, 1000, 100);
				//xsprintf(tx_buf,"%4d -----\n",tripmeter());
				//ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			}
			pattern = 101;
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


//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 TeraTermなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i< BT_MAX_RX_BUF_SIZE; i++) {
		rx_buf[i] = 0; /* 受信バッファをクリア */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0) {
		/* 受信データあり */
		if (rx_buf[0] == 'i' || rx_buf[0] == 'I') {
			start = IN; /* IN 走行開始 */
		}
		if (rx_buf[0] == 'o' || rx_buf[0] == 'O') {
			start = OUT; /* OUT 走行開始 */
		}
		if (rx_buf[0] == 't' || rx_buf[0] == 'T') {
			start = TEST; /* OUT 走行開始 */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* 受信データをエコーバック */
		xsprintf(tx_buf,"\n course = %d\n",start);
		ecrobot_send_bt(tx_buf, 0, strlen(tx_buf));
	}

	return start;
}

#if 1

#endif



//*****************************************************************************
// 関数名 : strlen
// 引数 :文字列へのポインタ
// 返り値 : 文字数
// 概要 : 文字数カウント
//*****************************************************************************

int strlen(const char *s)
{
    int len = 0;

    while (*s++) len++;

    return (len);
}

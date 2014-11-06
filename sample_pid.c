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
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 2.5*/
#define PWM_ABS_MAX         60 /* 完全停止用モータ制御PWM絶対最大値 60*/
/* sample_c4マクロ */

#define CMD_START         '1'    /* リモートスタートコマンド(変更禁止) */
/* PID制御マクロ */
#define DELTA_T 0.004	//処理周期(4ms)
#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1

/* インコース、アウトコース */
#define CMAX 10
#define IN 1
#define OUT 2
#define TEST 3 //テストコース

#define TH(b,w)  ((b + w) / 2)	//(522)
#define TH2(b,w) ((b + w) / 2)	//(582)

/* 関数プロトタイプ宣言 */

static void tail_control(signed int angle);
static int remote_start(void);
static float pid_control(int sensor_val, int target_val);
static float math_limit(float val, float min, float max);
static void calibration(int *black,int *white,int angle);
static void line_follow(int speed, int turn, int gyro);
static void line_follow2(int speed, int max, int min);
static void line_follow3(int speed, int max, int min);
static void turn_left_gyro(int speed, int turn, int gyro);
static void turn_right_gyro(int speed, int turn, int gyro);
static int check_marker(int turn);
static int tripmeter(void);
static int tripmeter_left(void);
static int tripmeter_right(void);
static int strlen(const char *s);
static int check_Seesaw(void);
static int check_course(int distance);
static void check_position(void);

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];
char tx_buf[128];

/* 自己位置 */
typedef struct {
	float x; // X座標(mm)
	float y; // Y座標(mm)
	float dir; //進行方向(rad)
	int l_enc; //左エンコーダ地
	int r_enc; //右エンコーダ値
} POSITION_t;


/* コースデータ */
typedef struct {
	// S:直線 U:登り坂 D:下り坂 L:左カーブ R:右カーブ E:その他 M:マーカ X:終点
    char state;
    int distance;
    int speed;
} ET_COURSE_t;

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

POSITION_t now, prev = {0.0, 0.0, 0.0, 0, 0};

ET_COURSE_t in_course[CMAX] = {
	{'S',0,40},
	{'U',1806,50},
	{'D',2378,30},
	{'S',3518,50},
	{'L',3858,40},
	{'S',5114,50},
	{'E',6424,40},
	{'M',17270,20},
	{'G',19586,20},
	{'X',19956,0}
};

ET_COURSE_t out_course[CMAX] = {
	{'S',0,40},
	{'U',1020,50},
	{'D',1592,30},
	{'S',2732,50},
	{'L',3072,40},
	{'S',4722,50},
	{'E',6018,40},
	{'M',18276,20},
	{'G',19566,20},
	{'X',19936,0}
};

ET_COURSE_t test_course[CMAX] = {
	{'S',0,50},
	{'M',2000,50},
	{'M',2850,50},
	{'X',3000,0},
	{'X',3001,0},
	{'X',3002,0},
	{'X',3003,0},
	{'X',3004,0},
	{'X',3005,0},
	{'X',3006,0}
};

DATA_LOG_t data_log;

/* PID制御用 */
//static signed int diff[2];
//static float integral;

static int pattern = 0; /* ロボットの状態 */
static int course = 0; /* 走行するコース IN or OUT */
static int sonar = 255; //超音波センサ(無探知は255)
static int turn = 0, speed = 0; // 旋回速度、走行速度
static int light_sensor; //超音波センサ(無探知は255)
static int navi = 0, navi0 = 0;
static int gyro_sensor = 255; // ジャイロセンサの値

static unsigned int counter=0; /* TaskLoggerにより 50ms ごとにカウントアップ */
static unsigned int cnt_ms=0; /* OSEKフック関数により 1ms？ ごとにカウントアップ */

static float kp = KP;

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate()
{
	nxt_motor_set_speed(NXT_PORT_C,0,0);
	nxt_motor_set_speed(NXT_PORT_B,0,0);
	nxt_motor_set_speed(NXT_PORT_A,0,0);

	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
}

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
			if (check_Seesaw()>2) {
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
			if( check_Seesaw() > 2 ){
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
			if( check_Seesaw()>2 ){
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
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* 比例制御 */
	/* PWM出力飽和処理 */
	if (pwm > PWM_ABS_MAX)	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX) {
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}

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

//*****************************************************************************
// 関数名 : line_follow
// 引数 : speed, turn 走行速度、旋回速度
// 引数 : gyro_sensor　ジャイロセンサー値
// 返り値 : 無し
// 概要 : ライントレース
//*****************************************************************************
void line_follow(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
}

//*****************************************************************************
// 関数名 : line_follow2
// 引数 : Black (黒のセンサ値)
// 引数 : white (白のセンサ値)
// 返り値 : 無し
// 概要 : バランサーを使用しないライントレース
//*****************************************************************************
static void line_follow2(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;
	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn = 50;
	if (turn2 < -50) turn  = -50;
	pwm_L = speed - turn2;
	pwm_R = speed + turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
	//xsprintf(tx_buf,"lf2:turn2=%d,%d\n",turn2,ecrobot_get_light_sensor(NXT_PORT_S3));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
}

static void line_follow3(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;
	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn = 50;
	if (turn2 < -50) turn  = -50;
	pwm_L = speed + turn2;
	pwm_R = speed - turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
	xsprintf(tx_buf,"lf2:turn2=%d,%d\n",turn2,ecrobot_get_light_sensor(NXT_PORT_S3));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
}

//*****************************************************************************
// 関数名 : line_follow
// 引数 : speed, turn 走行速度、旋回速度
// 引数 : gyro_sensor　ジャイロセンサー値
// 返り値 : 無し
// 概要 : ライントレース
//*****************************************************************************
void turn_left_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L-50, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R+40, 1); /* 右モータPWM出力セット(-100～100) */
}
void turn_right_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // 左右モータPWM出力
	balance_control(
		(float)speed,								 /* 前後進命令(+:前進, -:後進) */
		(float)turn,								 /* 旋回命令(+:右旋回, -:左旋回) */
		(float)gyro_sensor, /* ジャイロセンサ値 */
		(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
		&pwm_L,										 /* 左モータPWM出力値 */
		&pwm_R									 /* 右モータPWM出力値 */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L+10, 1); /* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R-10, 1); /* 右モータPWM出力セット(-100～100) */
}

//*****************************************************************************
// 関数名 : check_marker
// 引数 : black (黒のセンサ値)
// 引数 : white (白のセンサ値)
// 返り値 : マーカー発見ならば１そうでないなら０
// 概要 : バランサーを使用しないライントレースTH(black, white)
//*****************************************************************************

static int check_marker(int turn)
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
static int tripmeter(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = (nxt_motor_get_count(NXT_PORT_C) + nxt_motor_get_count(NXT_PORT_B)); // エンコーダ左右合計
	return (((s / 360) * circumference) + (circumference * (s % 360) / 360)) / 2;
}

static int tripmeter_left(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = nxt_motor_get_count(NXT_PORT_C); // エンコーダ左
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}

static int tripmeter_right(void)
{
	int circumference  = 254; // 車輪円周長さ(mm)
	int s = nxt_motor_get_count(NXT_PORT_B); // エンコーダ右
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}

//*****************************************************************************
// 関数名 : check_Seesaw
// 引数 :なし
// 返り値 : 連続数
// 概要 : ジャイロセンサの値が一定以上連続した場合カウントする
//*****************************************************************************

static int check_Seesaw(void)
{
	static int prev_gyro, diff_gyro, cnt_gyro = 0;
	diff_gyro = gyro_sensor - prev_gyro;
	if (diff_gyro > 15) {cnt_gyro++;} else {cnt_gyro = 0;}
	prev_gyro = gyro_sensor;
	return cnt_gyro;
}

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

//*****************************************************************************
// 関数名 : check_position
// 引数 :なし
// 返り値 : グローバル変数x, yを更新
// 概要 : 自己位置座標推定
//*****************************************************************************

void check_position(void)
{
	int l_arc, r_arc;
	float l_distance, r_distance, distance;
	float theta;

	now.l_enc = nxt_motor_get_count(NXT_PORT_C);
	now.r_enc = nxt_motor_get_count(NXT_PORT_B);

	l_arc = now.l_enc - prev.l_enc;
	r_arc = now.r_enc - prev.r_enc;

	l_distance = 254.0 * l_arc / 360.0;
	r_distance = 254.0 * r_arc / 360.0;
	distance = (l_distance + r_distance) / 2.0;

	theta = (l_arc - r_arc) / 140.0;

	now.x = prev.x + (distance * cos(prev.dir + theta / 2.0));
	now.y = prev.y + (distance * sin(prev.dir + theta / 2.0));
	now.dir = theta + prev.dir;

	prev = now;
}

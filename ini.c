/*
 * ini.c
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
#include "ini.c"

#define DEVICE_NAME       "ET315"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */


//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize()
{
	/*************デバイス名を設定する***************/
	if(ecrobot_get_bt_status()==BT_NO_INIT){
		/**
		 * Bluetooth通信用デバイス名の変更は、Bluetooth通信接続が確立されていない場合のみ有効です。
		 * 通信接続確立時にはデバイス名は変更されません。(下記のAPIは何もしません)
		 */
		ecrobot_set_bt_device_name(DEVICE_NAME);
	}
	/************************************************/

	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); /* 完全停止用モータエンコーダリセット */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth通信初期化 */
}


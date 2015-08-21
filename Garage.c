/*
 * Garage.c
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
// 関数名 : garage
// 引数 : 無し
// 返り値 : 1(処理終了)/0(処理中)
// 概要 : ガレージインの動作
//*****************************************************************************
void garage( void )
{
	static unsigned int GG_pattern = 10;

	switch(GG_pattern){
		case 10: /*** 走行停止：尻尾を出しながら、一瞬加速 ***/
			GG_pattern = 20;
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,100,1);
			nxt_motor_set_speed(NXT_PORT_C,100,1);
			xsprintf(tx_buf,"%4d ---101101--\n",tripmeter());
			ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			systick_wait_ms(200);
			break;

		case 20: /*** 走行停止：左右モーター停止で静止状態に ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,0,1);
			nxt_motor_set_speed(NXT_PORT_C,0,1);
			break;
	}
}

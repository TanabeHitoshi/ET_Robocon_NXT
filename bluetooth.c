/*
 * bluetooth.c
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
#include "isLineSensor.h"
#include "isCourse.h"
#include "isPosition.h"
#include "bluetooth.h"

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];
char tx_buf[128];

//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 TeraTermなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
int remote_start(void)
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
			start = R_course; /* IN 走行開始 */
		}
		if (rx_buf[0] == 'o' || rx_buf[0] == 'O') {
			start = L_course; /* OUT 走行開始 */
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


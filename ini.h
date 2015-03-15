/*
 * ini.h
 *
 *  Created on: 2014/11/06
 *      Author: ｈｔ
 */

#ifndef INI_H_
#define INI_H_

extern int gyro_sensor; // ジャイロセンサの値
extern unsigned int counter; /* TaskLoggerにより 50ms ごとにカウントアップ */
extern unsigned int cnt_ms; /* OSEKフック関数により 1ms？ ごとにカウントアップ */
extern int pattern; /* ロボットの状態 */
extern int sonar; //超音波センサ(無探知は255)
extern int navi, navi0;

void ecrobot_device_initialize();	// デバイス初期化処理フック関数
void ecrobot_device_terminate();	// デバイス終了処理フック関数

#endif /* INI_H_ */

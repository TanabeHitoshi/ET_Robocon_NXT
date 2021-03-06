/*
 * isLineSensor.h
 *
 *  Created on: 2014/11/06
 *      Author: ｈｔ
 */

#ifndef ISLINESENSOR_H_
#define ISLINESENSOR_H_

/* PID制御マクロ */
#define DELTA_T 0.004	//処理周期(4ms)
#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1
/* インコース、アウトコース */
#define CMAX 10
#define R_course 1
#define L_course 2
#define TEST 3 //テストコース

extern float kp;
extern float ki;
extern float kd;
extern int course; /* 走行するコース IN or OUT */
extern int black, white; // 白の値，黒のセンサ値
extern int black2, white2; // 傾倒時の白黒のセンサ値

float pid_control(int sensor_val, int target_val);
void calibration(int *black,int *white,int angle);

#endif

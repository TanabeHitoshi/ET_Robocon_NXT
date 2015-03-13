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
#define IN 1
#define OUT 2
#define TEST 3 //テストコース

extern float kp;
extern int course; /* 走行するコース IN or OUT */

float pid_control(int sensor_val, int target_val);
void calibration(int *black,int *white,int angle);

#endif

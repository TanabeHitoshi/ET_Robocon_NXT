/*
 * isLineSensor.h
 *
 *  Created on: 2014/11/06
 *      Author: ����
 */

#ifndef ISLINESENSOR_H_
#define ISLINESENSOR_H_

/* PID����}�N�� */
#define DELTA_T 0.004	//��������(4ms)
#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1
/* �C���R�[�X�A�A�E�g�R�[�X */
#define CMAX 10
#define IN 1
#define OUT 2
#define TEST 3 //�e�X�g�R�[�X

extern float kp;
extern int course; /* ���s����R�[�X IN or OUT */

float pid_control(int sensor_val, int target_val);
void calibration(int *black,int *white,int angle);

#endif

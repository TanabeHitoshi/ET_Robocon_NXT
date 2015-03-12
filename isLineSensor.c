/*
 * isLineSensor.c
 *
 *  Created on: 2014/11/06
 *      Author: ����
 */

#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "isLineSensor.h"

float kp = KP;
int course = 0; /* ���s����R�[�X IN or OUT */
//*****************************************************************************
// �֐��� : math_limit
// ���� : �l�A�����l�A����l
// �߂�l : �����l���l<����l
// �T�v :
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
// �֐��� : pid_sample
// ���� : sensor_val (�Z���T�[�l), target_val(�ڕW�l)
// �Ԃ�l : �����
// �T�v :PID����T���v���i���L�̂Ƃ��납��̃R�s�[�j
// ET���{�R���ł͂��߂�V�X�e������i4�j
// ���炩�ň��肵�����C���g���[�X����������v
// http://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html
//*****************************************************************************

float pid_control(int sensor_val, int target_val)
{
	float p =0, i=0, d=0;

	static signed int diff[2] = {0, 0};
	static float integral = 0.0;

	diff[0] = diff[1];
	diff[1] = sensor_val - target_val;	//�΍����擾
	integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

	p = kp * diff[1];
	i = KI * integral;
	d = KD * (diff[1] - diff[0]) / DELTA_T;
	//xsprintf(tx_buf,"pid:s=%d,t=%d,pid=%d\n",sensor_val,target_val,(int)(p+i+d));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
	if (course == OUT) 	return  -math_limit(p + i + d, -100.0, 100.0);
	else return  math_limit(p + i + d, -100.0, 100.0);
}


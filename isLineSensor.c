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
#include "drive.h"
#include "isLineSensor.h"

float kp = KP;
float ki = KI;
float kd = KD;

int course = 0; /* ���s����R�[�X IN or OUT */
int black = 508, white = 664; // ���̒l�C���̃Z���T�l
int black2 = 559, white2 = 746; // �X�|���̔����̃Z���T�l

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
	i = ki * integral;
	d = kd * (diff[1] - diff[0]) / DELTA_T;
	//xsprintf(tx_buf,"pid:s=%d,t=%d,pid=%d\n",sensor_val,target_val,(int)(p+i+d));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
	if (course == L_course) 	return  -math_limit(p + i + d, -100.0, 100.0);
	else return  -math_limit(p + i + d, -100.0, 100.0);
}
//*****************************************************************************
// �֐��� : calibration
// ���� : *black (���A�ŏ��l)�C*white�i���A�ő�l�j
// �Ԃ�l : ����
// �T�v : ���Z���T�̎蓮�L�����u���[�V����
//        �����̏��Ń^�b�`����B
//*****************************************************************************
void calibration(int *black,int *white,int angle)
{
	while(1) {
		tail_control(angle);

		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			ecrobot_sound_tone(440, 170, 100);
			*black = ecrobot_get_light_sensor(NXT_PORT_S3);
			display_clear(0);		/* ��ʕ\�� */
			display_goto_xy(0, 1);
			display_string("BLACK:");
			display_int(*black, 4);
			break;
		}//if
		systick_wait_ms(100); /* 10msec�E�F�C�g */
	}//while
	display_update();
	while(ecrobot_get_touch_sensor(NXT_PORT_S4));
	ecrobot_sound_tone(880, 170, 100);

	while(1) {
		tail_control(angle);
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			ecrobot_sound_tone(880, 170, 100);
			*white = ecrobot_get_light_sensor(NXT_PORT_S3);
			//display_clear(0);		/* ��ʕ\�� */
			display_goto_xy(0, 2);
			display_string("WHITE:");
			display_int(*white, 4);
			break;
		}//if
		systick_wait_ms(100); /* 10msec�E�F�C�g */
	}//while

	//display_clear(0);		/* ��ʕ\�� */
	display_goto_xy(0,4);
	display_string("TH:");
	display_int(TH(*black,*white), 3);
	display_update();
	while(ecrobot_get_touch_sensor(NXT_PORT_S4));
	ecrobot_sound_tone(440, 170, 100);
}


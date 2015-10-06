/*
 * drive.c
 *
 *  Created on: 2014/11/09
 *      Author: ����
 */
#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "ini.h"
#include "drive.h"

int turn = 0, speed = 0; // ���񑬓x�A���s���x

//*****************************************************************************
// �֐��� : line_follow
// ���� : speed, turn ���s���x�A���񑬓x
// ���� : gyro_sensor�@�W���C���Z���T�[�l
// �Ԃ�l : ����
// �T�v : ���C���g���[�X
//*****************************************************************************
void line_follow(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // ���E���[�^PWM�o��
	balance_control(
		(float)speed,								 /* �O��i����(+:�O�i, -:��i) */
		(float)turn,								 /* ���񖽗�(+:�E����, -:������) */
		(float)gyro_sensor, /* �W���C���Z���T�l */
		(float)GYRO_OFFSET,							 /* �W���C���Z���T�I�t�Z�b�g�l */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
		&pwm_L,										 /* �����[�^PWM�o�͒l */
		&pwm_R									 /* �E���[�^PWM�o�͒l */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
}
//*****************************************************************************
// �֐��� : line_follow2
// ���� : Black (���̃Z���T�l)
// ���� : white (���̃Z���T�l)
// �Ԃ�l : ����
// �T�v : �o�����T�[���g�p���Ȃ����C���g���[�X
//*****************************************************************************
void line_follow2(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;
	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn2 = 50;
	if (turn2 < -50) turn2  = -50;
	pwm_L = speed - turn2;
	pwm_R = speed + turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
	//xsprintf(tx_buf,"lf2:turn2=%d,%d\n",turn2,ecrobot_get_light_sensor(NXT_PORT_S3));
	//ecrobot_send_bt(tx_buf,0,strlen(tx_buf));
}

void line_follow3(int speed, int black, int white)
{
	int pwm_L, pwm_R, turn2;

	turn2 = KP * (ecrobot_get_light_sensor(NXT_PORT_S3) - TH2(black, white));
	if (turn2 > 50) turn2 = 50;
	if (turn2 < -50) turn2  = -50;
	pwm_L = speed + turn2;
	pwm_R = speed - turn2;
	if (pwm_L > 100) pwm_L = 100;
	if (pwm_L < -100) pwm_L  = -100;
	if (pwm_R > 100) pwm_R = 100;
	if (pwm_R < -100) pwm_R  = -100;
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
}
//*****************************************************************************
// �֐��� : turn_left_follow
// ���� : speed, turn ���s���x�A���񑬓x
// ���� : gyro_sensor�@�W���C���Z���T�[�l
// �Ԃ�l : ����
// �T�v : ���C���g���[�X
//*****************************************************************************
void turn_left_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // ���E���[�^PWM�o��
	balance_control(
		(float)speed,								 /* �O��i����(+:�O�i, -:��i) */
		(float)turn,								 /* ���񖽗�(+:�E����, -:������) */
		(float)gyro_sensor, /* �W���C���Z���T�l */
		(float)GYRO_OFFSET,							 /* �W���C���Z���T�I�t�Z�b�g�l */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
		&pwm_L,										 /* �����[�^PWM�o�͒l */
		&pwm_R									 /* �E���[�^PWM�o�͒l */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L-50, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R+40, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
}
//*****************************************************************************
// �֐��� : turn_right_follow
// ���� : speed, turn ���s���x�A���񑬓x
// ���� : gyro_sensor�@�W���C���Z���T�[�l
// �Ԃ�l : ����
// �T�v : ���C���g���[�X
//*****************************************************************************
void turn_right_gyro(int speed, int turn, int gyro_sensor)
{
	signed char pwm_L, pwm_R; // ���E���[�^PWM�o��
	balance_control(
		(float)speed,								 /* �O��i����(+:�O�i, -:��i) */
		(float)turn,								 /* ���񖽗�(+:�E����, -:������) */
		(float)gyro_sensor, /* �W���C���Z���T�l */
		(float)GYRO_OFFSET,							 /* �W���C���Z���T�I�t�Z�b�g�l */
		(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
		(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
		&pwm_L,										 /* �����[�^PWM�o�͒l */
		&pwm_R									 /* �E���[�^PWM�o�͒l */
	);
	nxt_motor_set_speed(NXT_PORT_C, pwm_L+10, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R-10, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* ��ᐧ�� */
	/* PWM�o�͖O�a���� */
	if (pwm > PWM_ABS_MAX)	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX) {
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}


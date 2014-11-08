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

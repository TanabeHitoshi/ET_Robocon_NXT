/*
 * isPositione.c
 *
 *  Created on: 2015/03/13
 *      Author: ����
 */

#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "ini.h"
#include "isLineSensor.h"
#include "isPosition.h"

//*****************************************************************************
// �֐��� : tripmeter
// ���� : �Ȃ�	NXT_PORT_C(��), NXT_PORT_B(�E)
// �Ԃ�l : ���s�����i�����j
// �T�v : �G���R�[�_�[�ő��s�����𑪒肷��
//*****************************************************************************
int tripmeter(void)
{
	int circumference  = 254; // �ԗ։~������(mm)
	int s = (nxt_motor_get_count(NXT_PORT_C) + nxt_motor_get_count(NXT_PORT_B)); // �G���R�[�_���E���v
	return (((s / 360) * circumference) + (circumference * (s % 360) / 360)) / 2;
}

int tripmeter_left(void)
{
	int circumference  = 254; // �ԗ։~������(mm)
	int s = nxt_motor_get_count(NXT_PORT_C); // �G���R�[�_��
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}

int tripmeter_right(void)
{
	int circumference  = 254; // �ԗ։~������(mm)
	int s = nxt_motor_get_count(NXT_PORT_B); // �G���R�[�_�E
	return ((s / 360) * circumference) + (circumference * (s % 360) / 360);
}
//*****************************************************************************
// �֐��� : check_position
// ���� :�Ȃ�
// �Ԃ�l : �O���[�o���ϐ�x, y���X�V
// �T�v : ���Ȉʒu���W����
//*****************************************************************************
void check_position(void)
{
	int l_arc, r_arc;
	float l_distance, r_distance, distance;
	float theta;

	now.l_enc = nxt_motor_get_count(NXT_PORT_C);
	now.r_enc = nxt_motor_get_count(NXT_PORT_B);

	l_arc = now.l_enc - prev.l_enc;
	r_arc = now.r_enc - prev.r_enc;

	l_distance = 254.0 * l_arc / 360.0;
	r_distance = 254.0 * r_arc / 360.0;
	distance = (l_distance + r_distance) / 2.0;

	theta = (l_arc - r_arc) / 140.0;

	now.x = prev.x + (distance * cos(prev.dir + theta / 2.0));
	now.y = prev.y + (distance * sin(prev.dir + theta / 2.0));
	now.dir = theta + prev.dir;

	prev = now;
}

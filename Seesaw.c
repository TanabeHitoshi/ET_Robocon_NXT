/*
 * Seesaw.c
 *
 *  Created on: 2015/03/11
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
#include "Seesaw.h"

//*****************************************************************************
// �֐��� : check_Seesaw
// ���� :�Ȃ�
// �Ԃ�l : �A����
// �T�v : �W���C���Z���T�̒l�����ȏ�A�������ꍇ�J�E���g����
//*****************************************************************************

int check_Seesaw(int gyro_sensor)
{
	static int prev_gyro, diff_gyro, cnt_gyro = 0;
	diff_gyro = gyro_sensor - prev_gyro;
	if (diff_gyro > 15) {cnt_gyro++;} else {cnt_gyro = 0;}
	prev_gyro = gyro_sensor;
	return cnt_gyro;
}

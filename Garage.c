/*
 * Garage.c
 *
 *  Created on: 2015/03/15
 *      Author: ����
 */

#include "xprintf.h"
#include "math.h"
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "ini.h"
#include "Seesaw.h"
#include "drive.h"
#include "isLineSensor.h"
#include "isCourse.h"
#include "isPosition.h"
#include "bluetooth.h"
#include "LookUpGate.h"

//*****************************************************************************
// �֐��� : garage
// ���� : ����
// �Ԃ�l : 1(�����I��)/0(������)
// �T�v : �K���[�W�C���̓���
//*****************************************************************************
void garage( void )
{
	static unsigned int GG_pattern = 10;

	switch(GG_pattern){
		case 10: /*** ���s��~�F�K�����o���Ȃ���A��u���� ***/
			GG_pattern = 20;
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,100,1);
			nxt_motor_set_speed(NXT_PORT_C,100,1);
			xsprintf(tx_buf,"%4d ---101101--\n",tripmeter());
			ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			systick_wait_ms(200);
			break;

		case 20: /*** ���s��~�F���E���[�^�[��~�ŐÎ~��Ԃ� ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,0,1);
			nxt_motor_set_speed(NXT_PORT_C,0,1);
			break;
	}
}

/*
 * ini.c
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
#include "ini.h"

#define DEVICE_NAME       "ET315"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY          "1234" /* Bluetooth�ʐM�p�p�X�L�[ */

/* OSEK declarations */
DeclareCounter(SysTimerCnt);

int gyro_sensor = 255; // �W���C���Z���T�̒l
unsigned int counter=0; /* TaskLogger�ɂ�� 50ms ���ƂɃJ�E���g�A�b�v */
unsigned int cnt_ms=0; /* OSEK�t�b�N�֐��ɂ�� 1ms�H ���ƂɃJ�E���g�A�b�v */
int pattern = 0; /* ���{�b�g�̏�� */
int sonar = 255; //�����g�Z���T(���T�m��255)
int navi = 0, navi0 = 0;

//*****************************************************************************
// �֐��� : ecrobot_device_initialize
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�����������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_initialize()
{
	/*************�f�o�C�X����ݒ肷��***************/
	if(ecrobot_get_bt_status()==BT_NO_INIT){
		/**
		 * Bluetooth�ʐM�p�f�o�C�X���̕ύX�́ABluetooth�ʐM�ڑ����m������Ă��Ȃ��ꍇ�̂ݗL���ł��B
		 * �ʐM�ڑ��m�����ɂ̓f�o�C�X���͕ύX����܂���B(���L��API�͉������܂���)
		 */
		ecrobot_set_bt_device_name(DEVICE_NAME);
	}
	/************************************************/

	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* ���Z���T�ԐFLED��ON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)�������� */
	nxt_motor_set_count(NXT_PORT_A, 0); /* ���S��~�p���[�^�G���R�[�_���Z�b�g */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth�ʐM������ */
}
//*****************************************************************************
// �֐��� : ecrobot_device_terminate
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�I�������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_terminate()
{
	nxt_motor_set_speed(NXT_PORT_C,0,0);
	nxt_motor_set_speed(NXT_PORT_B,0,0);
	nxt_motor_set_speed(NXT_PORT_A,0,0);

	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* ���Z���T�ԐFLED��OFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)���I�� */
	ecrobot_term_bt_connection(); /* Bluetooth�ʐM���I�� */
}
//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//*****************************************************************************
/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;

	cnt_ms++;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}



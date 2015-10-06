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
#include "Seesaw.h"
#include "drive.h"
#include "isLineSensor.h"
#include "isCourse.h"
#include "isPosition.h"
#include "bluetooth.h"
#include "LookUpGate.h"

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
	if (diff_gyro > 30) {cnt_gyro++;} else {cnt_gyro = 0;}
	prev_gyro = gyro_sensor;
	return cnt_gyro;
}

//*****************************************************************************
// �֐��� : seesaw
// ���� :�Ȃ�
// �Ԃ�l : 1(�����I��)/0(������)
// �T�v : �V�[�\�[��̓���
//*****************************************************************************
int seesaw(void)
{
	static unsigned int SS_pattern = 10;

	switch(SS_pattern){
		case 10: /*** �V�[�\�[�F200mm���x�𗎂Ƃ��ăA�v���[�` ***/
			//xsprintf(tx_buf,"%4d, %3d\n",tripmeter(),gyro_sensor);
			//ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			//if (tripmeter() - measure0 > 180 ) {
			//	counter = 0;
			//	measure0 = tripmeter();
			//	pattern = 31;
			//}
			if (check_Seesaw(gyro_sensor)>2) {
				ecrobot_sound_tone(440*3, 100, 100);
				measure0 = tripmeter();
				counter = 0;
				SS_pattern = 20;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20: /*** �V�[�\�[�F150mm�X�s�[�h�������ăV�[�\�[�ɏ��グ�� ***/
			if (tripmeter() - measure0 > 30 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 30;
			}
			speed = 70;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 30: /*** �V�[�\�[�F250mm�������o�� ***/
			if (tripmeter() - measure0 > 450 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 40;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 40: /*** �V�[�\�[�F70mm����̓u���[�L�� ***/
			if (tripmeter() - measure0 > 70 ) {
				counter = 0;
				measure0 = tripmeter();
				SS_pattern = 50;
			}
			speed = -50;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 50: /*** �V�[�\�[�F200mm�p�������肷��܂ł������i�� ***/
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
			}
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			break;
	}
	if(SS_pattern == 50)
		return 1;	/* �V�[�\�[����I�� */
	else
		return 0;	/* �V�[�\�[���쒆 */
}

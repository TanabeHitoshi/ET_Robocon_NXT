/*
 * LookUpGate.c
 *
 *  Created on: 2015/03/14
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

//*****************************************************************************
// �֐��� : lookupgate
// ���� : ����
// �Ԃ�l : 1(�����I��)/0(������)
// �T�v : ���b�N�A�b�v�Q�[�g����
//*****************************************************************************
int lookupgate( void )
{
	static unsigned int LUT_pattern = 10;
	signed int fangle = 50; // �X�|���̃I�t�Z�b�g�p

	pattern = pattern * 100 +LUT_pattern;

	switch(LUT_pattern){
		case 10:
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(440, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				LUT_pattern =20;
			}
			break;

		case 20: /*** ���b�N�A�b�v�Q�[�g�F���ɌX�| ***/
			speed = 30;
			//tail_control(TAIL_ANGLE_STAND_UP);  // �K�����o��
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4�@�ŋ����I�Ɍ��ɌX����
			//ecrobot_sound_tone(660, 1, 100);
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

			if (counter > 1) { // 50ms * 5 ���̏�Ԃ�ۂ�
				counter = 0;
				LUT_pattern = 30;
			}
			break;

		case 30: /*** ���b�N�A�b�v�Q�[�g�F�K�����������p�x�����炵�āA���s�̂�Q���� ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				LUT_pattern =40;
				measure0 = tripmeter();
			}
			break;

		case 40:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (counter > 30 ) {
				counter = 0;
				LUT_pattern =50;
				measure0 = tripmeter();
			}
			break;

		case 50:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 30;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
				LUT_pattern =60;
			}
			break;

		case 60:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = -30;
			nxt_motor_set_speed(NXT_PORT_C, speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B, speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

		//			line_follow3(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 < -350 ) {
				counter = 0;
				LUT_pattern =70;
				measure0 = tripmeter();
			}
			break;

		case 70:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 30;

			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				LUT_pattern =80;
				measure0 = tripmeter();
			}
			break;

		case 80:/***  ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle + counter/2 );
			if (counter > 64 ) {
					counter = 0;
					LUT_pattern =90;
					measure0 = tripmeter();
				}
			break;

		case 90:/***  ***/
			speed = 20;
			line_follow2(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 420 ) {
				counter = 0;
				LUT_pattern =10;
				measure0 = tripmeter();
				}
			break;
	}

	pattern = pattern /100;

	if(LUT_pattern == 90)
		return 1;	/* ���b�N�A�b�v�Q�[�g����I�� */
	else
		return 0;	/* ���b�N�A�b�v�Q�[�g���쒆 */

}

/*
 * Staris.c
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
// �֐��� : staris
// ���� : ����
// �Ԃ�l : 1(�����I��)/0(������)
// �T�v : �K�i�̓���
//*****************************************************************************
int stairs( void )
{

	switch(ST_pattern){
		case 10:/* �i�����O�̃g���[�X ���x�𗎂Ƃ��đ��s */
			speed = 20;//20
			kp = 0.5;
//			tail_control(TAIL_ANGLE_STAND_UP - 25);
			if( check_Seesaw(gyro_sensor) > 2 ){
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 20;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20:/* �i�����m�A���x���グ�ēo�� */
			speed = 200;
			line_follow(speed, 0, gyro_sensor + 10);
			if (tripmeter() - measure0 > 10 ){
				counter = 0;
				ST_pattern = 30;
			}
			break;

		case 30:/* �������Ƀo�b�N���đ̐��𐮂��� */
			speed = -10;
			line_follow(speed, 5, gyro_sensor);
			if (counter > 50 ){
				counter = 0;
//				ST_pattern = 40;
				ST_pattern = 90;
			}
			break;

		case 40:/* ���C���̌��o���Ȃ���O�� */
			kp = 0.3;
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				ST_pattern = 50;
			}			break;

		case 50:/* ���C���̌��o���Ȃ���O�� */
			kp = 0.5;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (counter > 200 ){
				counter = 0;
				ST_pattern = 60;
			}
			break;
		case 60:/* ���C���̌��o */
			kp = 0.3;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black ){
				counter = 0;
				ST_pattern = 70;
				measure0 = tripmeter_right();
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 70:/* �^�[�� */
			turn_left_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > (502 - 20) ){
				counter = 0;
				ST_pattern = 80;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 80:/* ���C���̌��o */
			turn_right_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > 60 ){
				counter = 0;
				ST_pattern = 90;
			}
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black){
				counter = 0;
				ST_pattern = 90;
			}
			break;

		case 90:/* �i���̌��m */
			kp = KP;
			speed = 40;
			if( check_Seesaw(gyro_sensor)>2 ){
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 100;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 100:/* �����O�ɐi�� */
			speed = 20;
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				ST_pattern = 110;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 110:/* �}�[�J�[��������*/
			speed = 40;
			line_follow(speed, turn, gyro_sensor);
			if (check_marker(turn)>0) {
				counter = 0;
				measure0 = tripmeter();
				ST_pattern = 120;

			}
			break;

		case 120:/* �e�[�������낷*/
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				ST_pattern =130;
			}
			break;

		case 130: /*** ���ɌX�| ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // �K�����o��
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4�@�ŋ����I�Ɍ��ɌX����
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

			if (counter > 1) { // 50ms * 5 ���̏�Ԃ�ۂ�
				counter = 0;
				ST_pattern =140;
			}
			break;

		case 140: /*** �K�����������p�x�����炵�āA���s�̂�Q���� ***/
			speed = 0;
			line_follow3(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				ST_pattern =150;
				measure0 = tripmeter();
			}
			break;

		case 150:/*** �Q�[�g�܂Ői�� ***/
			speed = 20;
			line_follow3(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 350 ) {
				counter = 0;
				measure0 = tripmeter();
				}
			break;
	}

	if(ST_pattern == 150)
		return 1;	/* �K�i����I�� */
	else
		return 0;	/* �K�i���쒆 */

}

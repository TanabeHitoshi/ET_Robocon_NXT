/*
 * ini.h
 *
 *  Created on: 2014/11/06
 *      Author: ����
 */

#ifndef INI_H_
#define INI_H_

extern int gyro_sensor; // �W���C���Z���T�̒l
extern unsigned int counter; /* TaskLogger�ɂ�� 50ms ���ƂɃJ�E���g�A�b�v */
extern unsigned int cnt_ms; /* OSEK�t�b�N�֐��ɂ�� 1ms�H ���ƂɃJ�E���g�A�b�v */
extern int pattern; /* ���{�b�g�̏�� */
extern int sonar; //�����g�Z���T(���T�m��255)
extern int navi, navi0;

void ecrobot_device_initialize();	// �f�o�C�X�����������t�b�N�֐�
void ecrobot_device_terminate();	// �f�o�C�X�I�������t�b�N�֐�

#endif /* INI_H_ */

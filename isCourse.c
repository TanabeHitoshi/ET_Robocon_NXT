/*
 * isCourse.c
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
#include "isCourse.h"

//*****************************************************************************
// �֐��� : check_marker
// ���� : black (���̃Z���T�l)
// ���� : white (���̃Z���T�l)
// �Ԃ�l : �}�[�J�[�����Ȃ�΂P�����łȂ��Ȃ�O
// �T�v : �o�����T�[���g�p���Ȃ����C���g���[�XTH(black, white)
//*****************************************************************************
int check_marker(int turn)


{
	static int r=0,l=0;
	if(turn < -20) {
		r++;
	} else {
		if (turn > 0) {
			r = 0;
		}
	}
	if(turn > 20) {
		l++;
	} else {
		if (turn < 0) {
			l = 0;
		}
	}

	//xsprintf(tx_buf,"%4d, %4d\n",turn,tripmeter());
	//ecrobot_send_bt(tx_buf,1, 12);

	if (r >= 5) {
		r = 0;
		ecrobot_sound_tone(440*4 , 10, 100);
		return 1; //�E�G�b�W���s�̂Ƃ��̓}�[�J�[�X�^�[�g
	}
	if (l >= 5) {
		l = 0;
		//ecrobot_sound_tone(440*2 , 10, 100);
		return -1;
	}
	return 0;
}
//*****************************************************************************
// �֐��� : check_course
// ���� : ����
// ���s�����Ō��݂̃R�[�X�󋵂��i�r
// �Ԃ�l : �z��̓Y����
//*****************************************************************************
int check_course(int distance)
{
	int i, found = CMAX - 1;

	for (i=0; i<CMAX; i++) {
		if (course == IN) { //in
			if (distance < in_course[i].distance) {
				found = i-1;
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		} else if (course == OUT){ //out
			if (distance < out_course[i].distance) {
				found = i-1;
				//printf("out\n");
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		} else {
			if (distance < test_course[i].distance) {
				found = i-1;
				//printf("out\n");
				//printf("found...%d trip:%d\n",found,distance);
				break;
			}
		}

	}

	return found;
}


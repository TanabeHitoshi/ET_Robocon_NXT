/*
 * bluetooth.c
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
#include "isLineSensor.h"
#include "isCourse.h"
#include "isPosition.h"
#include "bluetooth.h"

/* Bluetooth�ʐM�p�f�[�^��M�o�b�t�@ */
char rx_buf[BT_MAX_RX_BUF_SIZE];
char tx_buf[128];

//*****************************************************************************
// �֐��� : remote_start
// ���� : ����
// �Ԃ�l : 1(�X�^�[�g)/0(�ҋ@)
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B TeraTerm�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i< BT_MAX_RX_BUF_SIZE; i++) {
		rx_buf[i] = 0; /* ��M�o�b�t�@���N���A */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0) {
		/* ��M�f�[�^���� */
		if (rx_buf[0] == 'i' || rx_buf[0] == 'I') {
			start = R_course; /* IN ���s�J�n */
		}
		if (rx_buf[0] == 'o' || rx_buf[0] == 'O') {
			start = L_course; /* OUT ���s�J�n */
		}
		if (rx_buf[0] == 't' || rx_buf[0] == 'T') {
			start = TEST; /* OUT ���s�J�n */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* ��M�f�[�^���G�R�[�o�b�N */
		xsprintf(tx_buf,"\n course = %d\n",start);
		ecrobot_send_bt(tx_buf, 0, strlen(tx_buf));
	}

	return start;
}
//*****************************************************************************
// �֐��� : strlen
// ���� :������ւ̃|�C���^
// �Ԃ�l : ������
// �T�v : �������J�E���g
//*****************************************************************************

int strlen(const char *s)
{
    int len = 0;

    while (*s++) len++;

    return (len);
}


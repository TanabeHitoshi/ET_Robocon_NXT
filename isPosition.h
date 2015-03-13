/*
 * isCourse.h
 *
 *  Created on: 2015/03/13
 *      Author: ����
 */

#ifndef ISPOSITION_H_
#define ISPOSITION_H_

/* ���Ȉʒu */
typedef struct {
	float x; // X���W(mm)
	float y; // Y���W(mm)
	float dir; //�i�s����(rad)
	int l_enc; //���G���R�[�_�n
	int r_enc; //�E�G���R�[�_�l
} POSITION_t;

POSITION_t now, prev = {0.0, 0.0, 0.0, 0, 0};

int tripmeter(void);
int tripmeter_left(void);
int tripmeter_right(void);
void check_position(void);

#endif
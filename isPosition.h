/*
 * isCourse.h
 *
 *  Created on: 2015/03/13
 *      Author: ｈｔ
 */

#ifndef ISPOSITION_H_
#define ISPOSITION_H_

/* 自己位置 */
typedef struct {
	float x; // X座標(mm)
	float y; // Y座標(mm)
	float dir; //進行方向(rad)
	int l_enc; //左エンコーダ地
	int r_enc; //右エンコーダ値
} POSITION_t;

POSITION_t now, prev = {0.0, 0.0, 0.0, 0, 0};

extern int measure0; // 距離測定用（計測スタート地点）
extern int measure_L; // 距離測定用(左)（計測スタート地点）
extern int measure_R; // 距離測定用(右)（計測スタート地点）
extern int measure_P; // 距離測定用（難所のスタート地点）

int tripmeter(void);
int tripmeter_left(void);
int tripmeter_right(void);
void check_position(void);

#endif

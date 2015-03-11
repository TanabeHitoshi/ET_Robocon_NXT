/*
 * drive.h
 *
 *  Created on: 2014/11/09
 *      Author: ����
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#define GYRO_OFFSET 600 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) 600*/
#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1

#define TH(b,w)  ((b + w) / 2)	//(522)
#define TH2(b,w) ((b + w) / 2)	//(582)

void line_follow(int speed, int turn, int gyro);
void line_follow2(int speed, int black, int white);
void line_follow3(int speed, int black, int white);
void turn_left_gyro(int speed, int turn, int gyro_sensor);
void turn_right_gyro(int speed, int turn, int gyro_sensor);

#endif /* DRIVE_H_ */

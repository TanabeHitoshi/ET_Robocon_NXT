/*
 * drive.h
 *
 *  Created on: 2014/11/09
 *      Author: ����
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#define GYRO_OFFSET 600 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) 600*/

void line_follow(int speed, int turn, int gyro);

#endif /* DRIVE_H_ */

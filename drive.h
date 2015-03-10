/*
 * drive.h
 *
 *  Created on: 2014/11/09
 *      Author: ｈｔ
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#define GYRO_OFFSET 600 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) 600*/

void line_follow(int speed, int turn, int gyro);

#endif /* DRIVE_H_ */

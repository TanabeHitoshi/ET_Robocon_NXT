/*
 * isCourse.h
 *
 *  Created on: 2015/03/13
 *      Author: ｈｔ
 */

#ifndef ISCOURSE_H_
#define ISCOURSE_H_

/* コースデータ */
typedef struct {
	// S:直線 U:登り坂 D:下り坂 L:左カーブ R:右カーブ E:その他 M:マーカ X:終点
    char state;
    int distance;
    int speed;
} ET_COURSE_t;

ET_COURSE_t in_course[CMAX] = {
	{'S',0,40},
	{'U',1806,50},
	{'D',2378,30},
	{'S',3518,50},
	{'L',3858,40},
	{'S',5114,50},
	{'E',6424,40},
	{'M',17270,20},
	{'G',19586,20},
	{'X',19956,0}
};

ET_COURSE_t out_course[CMAX] = {
	{'S',0,40},
	{'U',1020,50},
	{'D',1592,30},
	{'S',2732,50},
	{'L',3072,40},
	{'S',4722,50},
	{'E',6018,40},
	{'M',18276,20},
	{'G',19566,20},
	{'X',19936,0}
};

ET_COURSE_t test_course[CMAX] = {
	{'S',0,50},
	{'M',2000,50},
	{'M',2850,50},
	{'X',3000,0},
	{'X',3001,0},
	{'X',3002,0},
	{'X',3003,0},
	{'X',3004,0},
	{'X',3005,0},
	{'X',3006,0}
};

int check_marker(int turn);
int check_course(int distance);

#endif

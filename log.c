/*
 * log.c
 *
 *  Created on: 2014/11/06
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
#include "Stairs.h"
#include "Garage.h"

/* OSEK declarations */
DeclareTask(TaskLogger);

/* log�p�\���� */
typedef struct {
	int distance;	//���s����
	int pattern;	//���s�p�^�[��
	int speed;		//���s���x
	int turn;		//�^�[�����x
	char state;		//�R�[�X���
	int x;			//�w���W
	int y;			//�x���W
	int dir;		//����
} DATA_LOG_t;

DATA_LOG_t data_log;

//*****************************************************************************
// �^�X�N�� : TaskLogger
// �T�v : ���K�[�^�X�N 50ms���ƂɋN��
// �����g�Z���T�[�A�J�E���^�[�C���N�������g�Axprintf�t���b�V��
//*****************************************************************************
/* Task1 executed every 50msec */
TASK(TaskLogger)
{
	counter++;

	//if (course == L_course || course == TEST || course == 0) {
		sonar = ecrobot_get_sonar_sensor(NXT_PORT_S2);
	//	if ( sonar < 0 ) sonar = 255;
	//} else {
	//	sonar = 255;
	//}

	if(pattern >= 10 && pattern <= 101) { //���s���̂Ƃ�
		navi = check_course(tripmeter());
//		if(navi0 != navi) ecrobot_sound_tone(440*4, 200, 100);

		data_log.distance = tripmeter();
		data_log.pattern = pattern;
		data_log.speed = speed;
		data_log.turn = turn;

		check_position();

		data_log.x = (int)now.x;
		data_log.y = (int)now.y;
		data_log.dir = (int)now.dir;

		if (course == R_course) {
			data_log.state = in_course[navi].state;
		} else if(course == L_course){
			data_log.state = out_course[navi].state;
		} else {
			data_log.state = test_course[navi].state;
		}
		navi0 = navi;

		xsprintf(tx_buf,"%6d,%6d,%4d,%4d,%c,%6d,%6d,%4d,%4d \n",
				data_log.distance,
				data_log.pattern,
				data_log.speed,
				data_log.turn,
				data_log.state,
				data_log.x,
				data_log.y,
				data_log.dir,
				sonar
		);


		ecrobot_send_bt(tx_buf,0,strlen(tx_buf));

	}
	/* send Sensor/Motors/NXT internal status to the host.
	 * NXT GamePad in the host PC accumulates all logging data
	 * and later you can save the logging data into a CSV file
	 */
	//ecrobot_bt_data_logger(i++, j--);

	/* display Sensors/Motors/NXT internal status */
	//ecrobot_status_monitor("Data Logging");
	//display_clear(0);		/* ��ʕ\�� */
	display_goto_xy(0,3);
	display_string("pattern:");
	display_int(pattern, 3);
	display_goto_xy(0,5);
	display_string("trip:");
	display_int(tripmeter(), 6);
	display_goto_xy(0,6);
	display_string("left :");
	display_int(nxt_motor_get_count(NXT_PORT_C), 6);
	display_goto_xy(0,7);
	display_string("right:");
	display_int(nxt_motor_get_count(NXT_PORT_B), 6);

	display_update();


	//ecrobot_sound_tone(880, 2, 100);
	TerminateTask();
}//TaskLogger

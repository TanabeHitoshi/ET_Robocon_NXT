/** ******************************************************************************
 **	�t�@�C���� : sample_pid.c
 **
 **	�T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/ATK1(OSEK)�pC�T���v���v���O����
 **
 ** ���L : (sample_c4��PID����version)
 ********************************************************************************/

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

DeclareTask(Task1);
DeclareTask(TaskLogger);

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET 600 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) 600*/
#define LIGHT_WHITE 500 /* ���F�̌��Z���T�l 500*/
#define LIGHT_BLACK 700 /* ���F�̌��Z���T�l 700*/
/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] 30*/
/* sample_c3�}�N�� */
/* sample_c4�}�N�� */
#define DEVICE_NAME       "ET315"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY          "1234" /* Bluetooth�ʐM�p�p�X�L�[ */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */

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

/* PID����p */

static int pattern = 0; /* ���{�b�g�̏�� */
static int sonar = 255; //�����g�Z���T(���T�m��255)
static int light_sensor; //�����g�Z���T(���T�m��255)
static int navi = 0, navi0 = 0;

//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{

//	signed int black = 508, white = 664; // ���̒l�C���̃Z���T�l
//	signed int black2 = 559, white2 = 746; // �X�|���̔����̃Z���T�l
	signed int tail_angle = 0, tail_cnt = 0; // TAIL�p�J�E���^ 
	unsigned int loop_start, loop_time; // ���[�v���Ԍv���p 


	pattern = 0; //�������

	xsprintf(tx_buf,"**** HELLO! ****\n");
	ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;


	while(1) {
		if (tripmeter() > 19800 && pattern < 100) pattern = 100;/*19.9m�ŃX�g�b�v*/

		loop_start = systick_get_ms();// while���[�v�̎�����4ms�ɕۂ��߂Ƀ��[�v�J�n�������L���B
		//4ms���ɃW���C���Z���T�A���Z���T�A��]�����A
		gyro_sensor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
		turn = pid_control(ecrobot_get_light_sensor(NXT_PORT_S3), TH(black, white));
		light_sensor = ecrobot_get_light_sensor(NXT_PORT_S3);

		//xsprintf(tx_buf,"%4d\n",gyro);
		//ecrobot_send_bt(tx_buf,0,6);

		switch (pattern) {

		case 0:  /*** �L�����u���[�V���� ***/
			ecrobot_sound_tone(880, 170, 100);
			 /* �蓮�L�����u���[�V���� */
			calibration(&white2, &black2, TAIL_ANGLE_STAND_UP - fangle);
			calibration(&white, &black, TAIL_ANGLE_STAND_UP);
			systick_wait_ms(500);
			pattern = 1;
			break;

		case 1: /*** �X�^�[�g�҂� ***/
			tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */

			if (turn < 5 && turn > -5) ecrobot_sound_tone(1500, 1, 30);
			
			course = remote_start();
			if (course  > 0 && sonar == 255) {
				course = OUT;
				ecrobot_sound_tone(880, 170, 100);
				pattern = 10;
				break;
			} else if (course > 0 ) {
				course = IN;
				ecrobot_sound_tone(220, 170, 100);
				pattern = 10;
				break; /* �����[�g�X�^�[�g */
			}

			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar < 150) {
				course = IN;
				ecrobot_sound_tone(880, 170, 100);
				systick_wait_ms(10); /* 10msec�E�F�C�g */
				pattern = 10;
				break; /* �^�b�`�Z���T�������ꂽ */
			}
			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar == 255) {
				course = OUT;
				ecrobot_sound_tone(440, 170, 100);
				systick_wait_ms(10); /* 10msec�E�F�C�g */
				pattern = 10;
				break; /* �^�b�`�Z���T�������ꂽ */
			}

			break;

		case 10: /*** ���s���� ***/
			tail_cnt = 0;
			tail_angle = TAIL_ANGLE_STAND_UP;
			balance_init();						/* �|���U�q���䏉���� */
			nxt_motor_set_count(NXT_PORT_C, 0); /* �����[�^�G���R�[�_���Z�b�g */
			nxt_motor_set_count(NXT_PORT_B, 0); /* �E���[�^�G���R�[�_���Z�b�g */
			/* �\�i�[�ڑ��m�F */

			systick_init();						/* ���s���ԃ��Z�b�g */
			pattern = 11;
			break;

		case 11: /*** �ʏ푖�s�� ***/
			// ���s�J�n����͑��x��}���đ��s���Ȃ���A�������e�[�����グ��
			tail_cnt++;
			if( tail_cnt > 10 && (tail_angle > 0) ){
				tail_cnt = 0;
				tail_angle--;
				speed = 10; // �e�[���A�b�v���͂��̑��x
			} else {
				speed = 100; // �ʏ�͂��̑��x
			}
			tail_control(tail_angle); // �o�����X���s�p�p�x�ɐ���

			if (sonar < 30 && tripmeter()> 11000 && course == OUT) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
			{
				pattern = 20;
				counter = 0;
				break;
			}
#if 1
			if((tripmeter() > 10200) && ( course == IN) ){
//			if((tripmeter() > 400) && ( course == IN) ){
			counter = 0;
					pattern = 40;
					ecrobot_sound_tone(440 * 3, 200, 100);
			}
#endif

			//if(tripmeter()==17000) ecrobot_sound_tone(440 * 3, 200, 100); // 17000
#if 0
			if (tripmeter() > 10000) { //�V�[�\�[���O�̃}�[�J�[�ɋ߂Â�����A�}�[�J�[���m 20000
				if (check_marker(turn)>0) {
					counter = 0;
					measure0 = tripmeter();
					pattern = 30;
				}
			}
#endif
			line_follow(speed, turn, gyro_sensor);
			break;

		case 20:	/* ���b�N�A�b�v�Q�[�g */
			if(lookupgate()){
				pattern = 100;
			}
			break;

		case 30: /* �V�[�\�[�@*/
			if(seesaw()){
				pattern = 100;
			}
			break;

		case 40:/* �i�����O�̃g���[�X���x�𗎂Ƃ��đ��s */
			if(stairs()){
				pattern = 100;
			}
			break;

		case 100: /* �K���[�W�C�� */
			garage();
			break;

		default:
			break;

		}//switch
 		
		/* �ł��邾��4msec�Ń��[�v */
		loop_time = systick_get_ms() - loop_start;
		if (loop_time<=4) {
			systick_wait_ms(4 - loop_time);
		}//
		//systick_wait_ms(4);
	}//while

}//TASK MAIN

///////////////////////////////////////////////////////////////////////////////

//*****************************************************************************
// �^�X�N�� : TaskLogger
// �T�v : ���K�[�^�X�N 50ms���ƂɋN��
// �����g�Z���T�[�A�J�E���^�[�C���N�������g�Axprintf�t���b�V��
//*****************************************************************************
/* Task1 executed every 50msec */
TASK(TaskLogger)
{
	counter++;

	//if (course == OUT || course == TEST || course == 0) {
		sonar = ecrobot_get_sonar_sensor(NXT_PORT_S2);
	//	if ( sonar < 0 ) sonar = 255;
	//} else {
	//	sonar = 255;
	//}

	if(pattern > 10 && pattern <= 101) { //���s���̂Ƃ�
		navi = check_course(tripmeter());
		if(navi0 != navi) ecrobot_sound_tone(440*4, 200, 100);

		data_log.distance = tripmeter();
		data_log.pattern = pattern;
		data_log.speed = speed;
		data_log.turn = turn;

		check_position();

		data_log.x = (int)now.x;
		data_log.y = (int)now.y;
		data_log.dir = (int)now.dir;

		if (course == IN) {
			data_log.state = in_course[navi].state;
		} else if(course == OUT){
			data_log.state = out_course[navi].state;
		} else {
			data_log.state = test_course[navi].state;
		}
		navi0 = navi;

		xsprintf(tx_buf,"%6d,%4d,%4d,%4d,%c,%6d,%6d,%4d,%4d \n",
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

///////////////////////////////////////////////////////////////////////////////



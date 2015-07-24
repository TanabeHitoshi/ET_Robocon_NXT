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
#include "log.h"

/* OSEK declarations */
DeclareTask(Task1);

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET 600 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) 600*/
#define LIGHT_WHITE 500 /* ���F�̌��Z���T�l 500*/
#define LIGHT_BLACK 700 /* ���F�̌��Z���T�l 700*/
/* sample_c2�}�N�� */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] 30*/
/* sample_c3�}�N�� */
#define DEVICE_NAME       "ET315"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY          "1234" /* Bluetooth�ʐM�p�p�X�L�[ */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */

static int light_sensor; //�����g�Z���T(���T�m��255)

//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{

	signed int tail_angle = 0, tail_cnt = 0; // TAIL�p�J�E���^ 
	unsigned int loop_start, loop_time; // ���[�v���Ԍv���p 
//	signed int fangle = 40; // �X�|���̃I�t�Z�b�g�p

	ecrobot_device_initialize();
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
				course = L_course;
				ecrobot_sound_tone(880, 170, 100);
				pattern = 10;
				break;
			} else if (course > 0 ) {
				course = R_course;
				ecrobot_sound_tone(220, 170, 100);
				pattern = 10;
				break; /* �����[�g�X�^�[�g */
			}

			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar < 150) {
				course = R_course;
				ecrobot_sound_tone(880, 170, 100);
				systick_wait_ms(10); /* 10msec�E�F�C�g */
				pattern = 10;
				break; /* �^�b�`�Z���T�������ꂽ */
			}
			if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1 && sonar == 255) {
				course = L_course;
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
				speed = 120; // �ʏ�͂��̑��x
				kp = 0.5;
				ki = KI;
				kd = KD;
			}
			tail_control(tail_angle); // �o�����X���s�p�p�x�ɐ���

	//		if (tripmeter()> 4134 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
		if (tripmeter()> 500 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
		{
				pattern = 30;
				counter = 0;
				ecrobot_sound_tone(880, 170, 100);
				break;
		}
//			if((tripmeter() > 10200) && ( course == R_course) ){
			if((tripmeter() > 2500) && ( course == R_course) ){
				counter = 0;
				pattern = 40;
				ecrobot_sound_tone(440 * 3, 200, 100);
			}
			//if(tripmeter()==17000) ecrobot_sound_tone(440 * 3, 200, 100); // 17000
			line_follow(speed, turn, gyro_sensor);
			break;

		case 12://L ��2���
			speed = 80; // �ʏ�͂��̑��x
			kp = KP;
			ki = KI;
			kd = KD;
			if (tripmeter()> 4710 && course == L_course)
			{
				pattern = 13;
				counter = 0;
				ecrobot_sound_tone(880, 170, 100);
				break;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 13://L ��3���
				speed = 60; // �ʏ�͂��̑��x
				kp = KP;
				ki = KI;
				kd = KD;
				if (tripmeter()> 5390 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
				{
					pattern = 14;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 14://L ��4���
				speed = 80; // �ʏ�͂��̑��x
				kp = KP;
				ki = KI;
				kd = KD;
				if (tripmeter()> 6534 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
				{
					pattern = 15;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 15://L ��5���
				speed = 60; // �ʏ�͂��̑��x
				kp = KP;
				ki = KI;
				kd = KD;
				if (tripmeter()> 7774 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
				{
					pattern = 16;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;
		case 16://L ��6���
				speed = 80; // �ʏ�͂��̑��x
				kp = KP;
				ki = KI;
				kd = KD;
				if (tripmeter()> 8937 && course == L_course) // ���b�N�A�b�v�Q�[�g���m�Ȃ�pattern=20 16000
				{
					pattern = 100;
					counter = 0;
					ecrobot_sound_tone(880, 170, 100);
					break;
				}
				line_follow(speed, turn, gyro_sensor);
				break;

		case 21://R ��2���
					speed = 20; // �ʏ�͂��̑��x
					kp = KP;
					ki = KI;
					kd = KD;
					if (tripmeter()> 4969 && course == R_course)
					{
						pattern = 22;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 22://R ��3���
					speed = 60; // �ʏ�͂��̑��x
					kp = KP;
					ki = KI;
					kd = KD;
					if (tripmeter()> 6294 && course == R_course)
					{
						pattern = 23;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 23://R ��4���
					speed = 20; // �ʏ�͂��̑��x
					kp = KP;
					ki = KI;
					kd = KD;
					if (tripmeter()> 7433 && course == R_course)
					{
						pattern = 24;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 24://R ��5���
					speed = 60; // �ʏ�͂��̑��x
					kp = KP;
					ki = KI;
					kd = KD;
					if (tripmeter()> 9019 && course == R_course)
					{
						pattern = 100;
						counter = 0;
						ecrobot_sound_tone(440, 170, 100);
						break;
					}
					line_follow(speed, turn, gyro_sensor);
					break;

		case 30:/* ���b�N�A�b�v�Q�[�g */
			line_follow(speed, turn, gyro_sensor);
			speed = 40;
			if(sonar < 30 ){
				speed = 0;
				counter = 0;
				pattern = 31;
			}
			break;
		case 31:	/* ���b�N�A�b�v�Q�[�g */
			if(lookupgate()){
					pattern = 100;
				}
				break;
		case 300: /* �V�[�\�[�@*/
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

}//TASK MAR_course

///////////////////////////////////////////////////////////////////////////////

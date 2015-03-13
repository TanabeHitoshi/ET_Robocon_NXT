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

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
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
#define TAIL_ANGLE_STAND_UP 105 /* ���S��~���̊p�x[�x] 108*/
#define TAIL_ANGLE_DRIVE      0 /* �o�����X���s���̊p�x[�x] 3*/
/* sample_c4�}�N�� */
#define DEVICE_NAME       "ET315"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY          "1234" /* Bluetooth�ʐM�p�p�X�L�[ */
#define CMD_START         '1'    /* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */
/* PID����}�N�� */

#define KP 0.7	//0.38
#define KI 0.0	//0.06
#define KD 0.1



/* �֐��v���g�^�C�v�錾 */
static int remote_start(void);
static int strlen(const char *s);

/* Bluetooth�ʐM�p�f�[�^��M�o�b�t�@ */
char rx_buf[BT_MAX_RX_BUF_SIZE];
char tx_buf[128];



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
//static signed int diff[2];
//static float integral;

static int pattern = 0; /* ���{�b�g�̏�� */

static int sonar = 255; //�����g�Z���T(���T�m��255)
static int turn = 0, speed = 0; // ���񑬓x�A���s���x
static int light_sensor; //�����g�Z���T(���T�m��255)
static int navi = 0, navi0 = 0;
static int gyro_sensor = 255; // �W���C���Z���T�̒l

static unsigned int counter=0; /* TaskLogger�ɂ�� 50ms ���ƂɃJ�E���g�A�b�v */
static unsigned int cnt_ms=0; /* OSEK�t�b�N�֐��ɂ�� 1ms�H ���ƂɃJ�E���g�A�b�v */


//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//*****************************************************************************
/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;

	cnt_ms++;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}

//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{

	signed int black = 508, white = 664; // ���̒l�C���̃Z���T�l 
	signed int black2 = 559, white2 = 746; // �X�|���̔����̃Z���T�l 
	signed int tail_angle = 0, tail_cnt = 0; // TAIL�p�J�E���^ 
	signed int fangle = 32; // �X�|���̃I�t�Z�b�g�p
	unsigned int loop_start, loop_time; // ���[�v���Ԍv���p 
	int measure0 = 0; // ��������p�i�v���X�^�[�g�n�_�j


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

		case 20:
			speed = 0;

			line_follow(speed, turn, gyro_sensor);

			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				pattern =201;
			}
			break;

		case 201: /*** ���b�N�A�b�v�Q�[�g�F���ɌX�| ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // �K�����o��
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4�@�ŋ����I�Ɍ��ɌX����
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

			if (counter > 1) { // 50ms * 5 ���̏�Ԃ�ۂ�
				counter = 0;
				pattern =21;
			}
			break;

		case 21: /*** ���b�N�A�b�v�Q�[�g�F�K�����������p�x�����炵�āA���s�̂�Q���� ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				pattern =22;
				measure0 = tripmeter();
			}
			break;

		case 22:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (counter > 30 ) {
				counter = 0;
				pattern =23;
				measure0 = tripmeter();
			}
			break;

		case 23:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 30;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 300 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern =24;
			}
			break;

		case 24:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = -20;
			nxt_motor_set_speed(NXT_PORT_C, speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B, speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

//			line_follow3(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 < -250 ) {
				counter = 0;
				pattern =25;
				measure0 = tripmeter();
			}
			break;
		case 25:/*** ���b�N�A�b�v�Q�[�g�F�X�|��Ԃ�30*50�~���b���C���g���[�X(���̏�ԂŃQ�[�g�ʉ�) ***/
			speed = 30;

			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle);
			if (tripmeter() - measure0 > 300 ) {
				counter = 0;
				pattern =26;
				measure0 = tripmeter();
			}
			break;

		case 26:/***  ***/
			speed = 0;
			line_follow2(speed, black2, white2);
			tail_control(TAIL_ANGLE_STAND_UP - fangle + counter/2 );
			if (counter > 64 ) {
					counter = 0;
					pattern =27;
					measure0 = tripmeter();
				}
			break;

		case 27:/***  ***/
			speed = 20;
			line_follow2(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 420 ) {
				counter = 0;
				pattern =101;
				measure0 = tripmeter();
				}
			break;

		case 30: /*** �V�[�\�[�F200mm���x�𗎂Ƃ��ăA�v���[�` ***/
			//xsprintf(tx_buf,"%4d, %3d\n",tripmeter(),gyro_sensor);
			//ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			//if (tripmeter() - measure0 > 180 ) {
			//	counter = 0;
			//	measure0 = tripmeter();
			//	pattern = 31;
			//}
			if (check_Seesaw(gyro_sensor)>2) {
				ecrobot_sound_tone(440*3, 100, 100);
				measure0 = tripmeter();
				counter = 0;
				pattern = 31;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 31: /*** �V�[�\�[�F150mm�X�s�[�h�������ăV�[�\�[�ɏ��グ�� ***/
			if (tripmeter() - measure0 > 30 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 32;
			}
			speed = 70;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 32: /*** �V�[�\�[�F250mm�������o�� ***/
			if (tripmeter() - measure0 > 450 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 33;
			}
			speed = 20;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 33: /*** �V�[�\�[�F70mm����̓u���[�L�� ***/
			if (tripmeter() - measure0 > 70 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 34;
			}
			speed = -50;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 34: /*** �V�[�\�[�F200mm�p�������肷��܂ł������i�� ***/
			if (tripmeter() - measure0 > 400 ) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 100;
			}
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			break;

		case 40:/* �i�����O�̃g���[�X���x�𗎂Ƃ��đ��s */
			speed = 20;
//			tail_control(TAIL_ANGLE_STAND_UP - 25);
			if( check_Seesaw(gyro_sensor) > 2 ){
				counter = 0;
				measure0 = tripmeter();
				pattern = 41;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 41:/* �i�����m�A���x���グ�ēo�� */
			speed = 100;
			line_follow(speed, 0, gyro_sensor + 10);
			if (tripmeter() - measure0 > 30 ){
				counter = 0;
				pattern = 42;
			}
			break;

		case 42:/* �������Ƀo�b�N���đ̐��𐮂��� */
			speed = -10;
			line_follow(speed, 5, gyro_sensor);
			if (counter > 50 ){
				counter = 0;
//				pattern = 43;
				pattern = 46;
			}
			break;

		case 43:/* ���C���̌��o���Ȃ���O�� */
			kp = 0.3;
			speed = 10;
			line_follow(speed, turn, gyro_sensor);
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				pattern = 431;
			}			break;

		case 431:/* ���C���̌��o���Ȃ���O�� */
			kp = 0.5;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (counter > 200 ){
				counter = 0;
				pattern = 44;
			}
			break;
		case 44:/* ���C���̌��o */
			kp = 0.3;
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black ){
				counter = 0;
				pattern = 45;
				measure0 = tripmeter_right();
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 45:/* �^�[�� */
			turn_left_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > (502 - 20) ){
				counter = 0;
				pattern = 455;
				ecrobot_sound_tone(880, 170, 100);
			}
			break;

		case 455:/* ���C���̌��o */
			turn_right_gyro(0, 0, gyro_sensor);
			if (tripmeter_right() - measure0 > 60 ){
				counter = 0;
				pattern = 46;
			}
			if (ecrobot_get_light_sensor(NXT_PORT_S3) > black){
				counter = 0;
				pattern = 46;
			}
			break;

		case 46:/* �i���̌��m */
			kp = KP;
			speed = 40;
			if( check_Seesaw(gyro_sensor)>2 ){
				counter = 0;
				measure0 = tripmeter();
				pattern = 47;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 47:/* �����O�ɐi�� */
			speed = 20;
			if (tripmeter() - measure0 > 200 ){
				counter = 0;
				pattern = 49;
			}
			line_follow(speed, turn, gyro_sensor);
			break;

		case 48:/* �}�[�J�[��������*/
			speed = 40;
			line_follow(speed, turn, gyro_sensor);
			if (check_marker(turn)>0) {
				counter = 0;
				measure0 = tripmeter();
				pattern = 49;

			}
			break;

		case 49:/* �e�[�������낷*/
			speed = 0;
			line_follow(speed, turn, gyro_sensor);
			tail_control(counter * 4 + 3);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter >= 20 ) {
				counter = 0;
				pattern =50;
			}
			break;

		case 50: /*** ���ɌX�| ***/
			speed = 20;
			//tail_control(TAIL_ANGLE_STAND_UP);  // �K�����o��
			//line_follow(speed, 0, GYRO_OFFSET + 2); // speed=0 turn=0 gyro_sensor = 4�@�ŋ����I�Ɍ��ɌX����
			nxt_motor_set_speed(NXT_PORT_C,speed, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
			nxt_motor_set_speed(NXT_PORT_B,speed, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */

			if (counter > 1) { // 50ms * 5 ���̏�Ԃ�ۂ�
				counter = 0;
				pattern =51;
			}
			break;

		case 51: /*** �K�����������p�x�����炵�āA���s�̂�Q���� ***/
			speed = 0;
			line_follow3(speed, black2, white2);
			//tail_control(TAIL_ANGLE_STAND_UP - fangle);
			tail_control(TAIL_ANGLE_STAND_UP - counter - 25);
			//ecrobot_sound_tone(880, 1, 100);
			if (counter > (fangle - 25) ) {
				counter = 0;
				pattern =52;
				measure0 = tripmeter();
			}
			break;

		case 52:/*** �Q�[�g�܂Ői�� ***/
			speed = 20;
			line_follow3(speed, black2, white2);
			//line_follow(speed, turn, gyro_sensor);
			tail_control(TAIL_ANGLE_STAND_UP);
			if (tripmeter() - measure0 > 350 ) {
				counter = 0;
				pattern =101;
				measure0 = tripmeter();
				}
			break;

		case 100: /*** ���s��~�F�K�����o���Ȃ���A��u���� ***/
			counter = 0;
			speed = 0;
			turn = 0;
			pattern = 101;
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			nxt_motor_set_speed(NXT_PORT_B,100,1);
			nxt_motor_set_speed(NXT_PORT_C,100,1);
			xsprintf(tx_buf,"%4d ---101101--\n",tripmeter());
			ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			systick_wait_ms(200);
			break;

		case 101: /*** ���s��~�F���E���[�^�[��~�ŐÎ~��Ԃ� ***/
			tail_control(TAIL_ANGLE_STAND_UP - 20);
			speed = 0;
			//ecrobot_sound_tone(440*3, 1000, 100);
			while (1) {
				tail_control(TAIL_ANGLE_STAND_UP - 20);
				nxt_motor_set_speed(NXT_PORT_B,0,1);
				nxt_motor_set_speed(NXT_PORT_C,0,1);
				//line_follow2(speed, black2, white2);
				//ecrobot_sound_tone(440*3, 1000, 100);
				//xsprintf(tx_buf,"%4d -----\n",tripmeter());
				//ecrobot_send_bt(tx_buf,0, strlen(tx_buf)) ;
			}
			pattern = 101;
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


//*****************************************************************************
// �֐��� : remote_start
// ���� : ����
// �Ԃ�l : 1(�X�^�[�g)/0(�ҋ@)
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B TeraTerm�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
static int remote_start(void)
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
			start = IN; /* IN ���s�J�n */
		}
		if (rx_buf[0] == 'o' || rx_buf[0] == 'O') {
			start = OUT; /* OUT ���s�J�n */
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

#if 1

#endif



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

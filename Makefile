# nxtOSEK���[�g�f�B���N�g��
NXTOSEK_ROOT = ../../nxtOSEK

# �^�[�Q�b�g���s�`���t�@�C����
TARGET = sample_c4_pid

# �C���N���[�h�p�X
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer

# ���C�u����
USER_LIB = nxtway_gs_balancer

# C�\�[�X�t�@�C��
TARGET_SOURCES = balancer_param.c sample_pid.c xprintf.c Seesaw.c ini.c drive.c isLineSensor.c isCourse.c isPosition.c

# TOPPERS/ATK(OSEK)�ݒ�t�@�C��
TOPPERS_OSEK_OIL_SOURCE = sample.oil

# ���L�̃}�N���͕ύX���Ȃ��ł�������
O_PATH ?= build

include $(NXTOSEK_ROOT)/ecrobot/ecrobot.mak

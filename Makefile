# nxtOSEKルートディレクトリ
NXTOSEK_ROOT = ../../nxtOSEK

# ターゲット実行形式ファイル名
TARGET = sample_c4_pid

# インクルードパス
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer

# ライブラリ
USER_LIB = nxtway_gs_balancer

# Cソースファイル
TARGET_SOURCES = balancer_param.c sample_pid.c xprintf.c Seesaw.c ini.c drive.c isLineSensor.c isCourse.c isPosition.c

# TOPPERS/ATK(OSEK)設定ファイル
TOPPERS_OSEK_OIL_SOURCE = sample.oil

# 下記のマクロは変更しないでください
O_PATH ?= build

include $(NXTOSEK_ROOT)/ecrobot/ecrobot.mak

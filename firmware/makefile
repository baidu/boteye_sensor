## Copyright Cypress Semiconductor Corporation, 2010-2011,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed
## by the license agreement included in the file
##
##      <install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
##

# [NOTE] FX3FWROOT should be defined outside of this makefile (in build.sh)

all:compile

include $(FX3FWROOT)/fw_build/fx3_fw/fx3_build_config.mak

MODULE = cyfxuvc

SOURCE= # $(MODULE).c
SOURCE= uvc.c	sensor_v034_raw.c camera_ptzcontrol.c\
	cyfxuvcdscr.c	\
	inv_icm20608.c  \
	i2c.c \
	extension_unit.c\
	fx3_bsp.c\
	kfifo.c\
	cyfxtx.c

ifeq ($(CYFXBUILD),arm)
SOURCE_ASM=cyfx_startup.S
else
SOURCE_ASM=cyfx_gcc_startup.S
endif

C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)

EXES = $(MODULE).$(EXEEXT)

#CCFLAGS += -DUVC_PTZ_SUPPORT

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT)
	$(LINK)

cyfxtx.c:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfxtx.c .

cyfx_startup.S:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfx_startup.S .

cyfx_gcc_startup.S:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfx_gcc_startup.S .

$(C_OBJECT) : %.o : %.c
	$(COMPILE)

$(A_OBJECT) : %.o : %.S
	$(ASSEMBLE)

clean:
	rm -f ./$(MODULE).$(EXEEXT)
	rm -f ./$(MODULE).map
	rm -f ./*.o
	rm -f cyfxtx.c cyfx_startup.S cyfx_gcc_startup.S
	rm -f ./$(MODULE).img


compile: $(C_OBJECT) $(A_OBJECT) $(EXES)

#[]#

##############################################################################
#
#   Objects Files
#
##############################################################################

OBJECTS+=${COMPILER}/main.o \
		 ${COMPILER}/stm32f7xx_it.o \
		 ${COMPILER}/stm32f7xx_hal_msp.o \
		 ${COMPILER}/log.o \
		 ${COMPILER}/system_stm32f7xx.o \
		 ${COMPILER}/ws281x.o \
		 ${COMPILER}/stripEffects.o \
		 ${COMPILER}/cy8cmbr3.o \
##############################################################################
#
#   The flags passed to the C compiler.
#
##############################################################################
CFLAGS_DEF+=

#==============================================================================
#
#                      Proprietary - Copyright (C) 2017
#------------------------------------------------------------------------------
# Supported MCUs      : STM32F
# Supported Compilers : GCC
#------------------------------------------------------------------------------
# File name         : Makefile
#
# project name      : MicArray firmware on STM32F7 using GCC
#
#
# Summary: www.cmq.vn - Embedded Sytem Articles, IOT Technology
#
#= History ====================================================================
# 1.0  12/08/2018  OLLI firmware Team
# - Creation
#============================================================================*/
#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************
PROJECT_NAME        ?=Olli-stm32f7-firmware
PROJECT_ROOT        ?=${PWD}
OUTPUT_DIR			?=Images
OBJ_BUILD			?=${OUTPUT_DIR}/obj
TARGET_DESC			?=${PROJECT_NAME}
OPT					?=s
CPU_NAME			?=cortex-m7

## Configure the st-flash directory for flashing or dump memory
ST_FLASH      		?= st-flash
ST_DUMP 			?= read ${OUTPUT_DIR}/${TARGET_DESC}.dump 0x20000000 0x2FFFF
##############################################################################
#
#   Common directories 
#
##############################################################################
COMPILER            ?=${OBJ_BUILD}

##############################################################################
#
#   Application Include Directories
#
##############################################################################

VPATH+=$(shell find ./ -type d  | grep -v ${OUTPUT_DIR} | sed 's/^/ :/' | tr -d '\n' | tr -d '\r')

RECURSIVE_MK = $(shell find ./ -name *.mk  | grep -v ${OUTPUT_DIR} | sed 's/^/ /' | tr -d '\n' | tr -d '\r')

INC_DIRS = $(shell find ./ -name *.mk  | grep -v ${OUTPUT_DIR} | sed 's/recursive.mk//' | sed 's/^/ -I /' | tr -d '\n' | tr -d '\r')


include ${RECURSIVE_MK}

#Link DSP library

LIBS = -lPDMFilter_CM7F_GCC -larm_cortexM7lfsp_math
###############################################################################
#
# Define CPU settings here
#
###############################################################################
FPU+=-mfloat-abi=hard \
     -mfpu=fpv5-sp-d16

CPU_DEF=-mcpu=${CPU_NAME} \
        -mthumb -mthumb-interwork -mlong-calls \
        ${FPU} \
        -O$(OPT) \
        -fsingle-precision-constant \
        -std=gnu99 \
        -Wstrict-prototypes \
        -ffunction-sections \
        -fdata-sections \
        -Wa,-adhlns=${COMPILER}/$(*F).lst \
        --specs=rdimon.specs \
        -fno-unroll-loops 
##############################################################################
#
#   The flags passed to the assembler.
#
##############################################################################
AFLAGS=-mcpu=${CPU_NAME} \
       -mthumb -mthumb-interwork -mlong-calls \
       ${FPU} \
       -O$(OPT) \
       -x assembler-with-cpp \
       -Wa,-amhls=${COMPILER}/$(*F).lst

##############################################################################
#
#   The command for calling the compiler.
#
##############################################################################

CC=arm-none-eabi-gcc

##############################################################################
#
#   The flags passed to the compiler.
#
##############################################################################
# C Define
CFLAGS_DEF += -DUSE_USB_FS
CFLAGS_DEF += -DUSE_IOEXPANDER
CFLAGS_DEF += -DUSE_STM32746G_DISCO
CFLAGS_DEF += -DDEBUG


CFLAGS += -g3 -gdwarf-2
# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

CFLAGS+= ${INC_DIRS} ${CFLAGS_DEF} ${CPU_DEF} 

##############################################################################
#
#   The command for calling the linker.
#
##############################################################################

LD=arm-none-eabi-gcc -mthumb -mcpu=${CPU_NAME} ${FPU}

##############################################################################
#
#    The flags passed to the linker.
#
##############################################################################

LDFLAGS+= -lc -u_printf_float 
LDFLAGS+= -Wl,-Map=${COMPILER}/${TARGET_DESC}.map,--cref,--gc-sections,--no-warn-mismatch 
LDFLAGS+= -o ${COMPILER}/${TARGET_DESC}.elf
LDFLAGS+= -L"Libraries/CMSIS/Lib/GCC"
##############################################################################
#
#    The command for extracting images from the linked executables.
#
##############################################################################

OBJCOPY=arm-none-eabi-objcopy

##############################################################################
#
#    The command for dumping output file
#
##############################################################################

OBJDUMP=arm-none-eabi-objdump
##############################################################################
#
#    The command for show size #
##############################################################################
OBJSIZE=arm-none-eabi-size

#******************************************************************************
#
# The rule for building the object file from each C source file.
#
#******************************************************************************

${COMPILER}/%.o: %.c
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  CC    ${<}";                                \
	 else                                                    \
	     echo ${CC} ${CFLAGS} -o ${@} -c ${<};               \
	 fi
	@${CC} ${CFLAGS} -o ${@} -c ${<}

#******************************************************************************
#
# The rule for building the object file from each assembly source file.
#
#******************************************************************************
${COMPILER}/%.o: %.s
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  CC    ${<}";                                \
	 else                                                    \
	     echo ${CC} ${AFLAGS} -o ${@} -c ${<};               \
	 fi
	@${CC} ${AFLAGS} -o ${@} -c ${<}
#******************************************************************************
#
# The rule for creating an object library.
#
#******************************************************************************
${COMPILER}/%.a:
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  AR    ${@}";                                \
	 else                                                    \
	     echo ${AR} -cr ${@} ${^};                           \
	 fi
	@${AR} -cr ${@} ${^}

#******************************************************************************
#
# The rule for linking the application.
#
#******************************************************************************
#sudo dd if=/dev/urandom of=./Encrypt.Key bs=1 count=1024 > /dev/null
${COMPILER}/%.elf:
	@${LD} -T${LINKER_SCRIPT_DEV} --entry \
	       ${ENTRY_${notdir ${@:.elf=}}} ${LDFLAGSgcc_${notdir ${@:.elf=}}} \
	       ${LDFLAGS} -o ${@} ${^}; echo "  LD    ${TARGET_DESC}"
	@${OBJCOPY} -O binary ${@} ${OUTPUT_DIR}/${TARGET_DESC}.bin ;\
	 ${OBJCOPY} -O ihex ${@} ${OUTPUT_DIR}/${TARGET_DESC}.hex 

$(shell mkdir -p ${OUTPUT_DIR} 2>/dev/null)
$(shell mkdir -p ${OBJ_BUILD} 2>/dev/null)

##############################################################################
#
#   The default rule, which causes init to be built.
#
##############################################################################

all: ${COMPILER}                    \
     ${COMPILER}/${TARGET_DESC}.elf \
     showsize 


showsize: ${COMPILER}      
	  	  @$(OBJSIZE) -B ${OUTPUT_DIR}/obj/$(TARGET_DESC).elf

clean:
	@rm -rf ${OUTPUT_DIR} ${COMPILER} ${wildcard *.bin} ${TARGET_DESC}.elf

flash:
	$(ST_FLASH) write ${OUTPUT_DIR}/${TARGET_DESC}.bin 0x08000000



dump-memory:
	$(ST_FLASH) $(ST_DUMP)

#
# The rule to create the target directory
#
${COMPILER}:
	@mkdir ${COMPILER} ${OUTPUT_DIR} 


${COMPILER}/${TARGET_DESC}.elf: ${OBJECTS} ${LIBS}
ENTRY_${TARGET_DESC}=${RESET_POINT}

#
#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__

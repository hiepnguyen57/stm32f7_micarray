##############################################################################
#
#   Objects Files
#
##############################################################################

OBJECTS+=${COMPILER}/stm32f7xx_hal.o \
         ${COMPILER}/stm32f7xx_hal_gpio.o \
         ${COMPILER}/stm32f7xx_hal_rcc.o \
		 ${COMPILER}/stm32f7xx_hal_rcc_ex.o \
         ${COMPILER}/stm32f7xx_hal_cortex.o \
         ${COMPILER}/stm32f7xx_hal_uart.o \
         ${COMPILER}/stm32f7xx_hal_pwr.o \
         ${COMPILER}/stm32f7xx_hal_pwr_ex.o \
		 ${COMPILER}/stm32f7xx_hal_i2c.o \
		 ${COMPILER}/stm32f7xx_hal_i2c_ex.o \
		 ${COMPILER}/stm32f7xx_hal_dma.o \
		 ${COMPILER}/stm32f7xx_hal_dma_ex.o \
		 ${COMPILER}/stm32f7xx_hal_dma2d.o \
		 ${COMPILER}/stm32f7xx_hal_tim.o \
		 ${COMPILER}/stm32f7xx_hal_tim_ex.o \
		 ${COMPILER}/stm32f7xx_hal_sai.o \
		 ${COMPILER}/stm32f7xx_hal_sai_ex.o \
		 ${COMPILER}/stm32f7xx_hal_i2s.o \
		 ${COMPILER}/stm32f7xx_hal_pcd.o \
		 ${COMPILER}/stm32f7xx_hal_pcd_ex.o \
		 ${COMPILER}/stm32f7xx_hal_sram.o \
		 ${COMPILER}/stm32f7xx_hal_sdram.o \
		 ${COMPILER}/stm32f7xx_hal_lptim.o \
		 ${COMPILER}/stm32f7xx_hal_hcd.o \
		 ${COMPILER}/stm32f7xx_hal_ltdc.o \
		 ${COMPILER}/stm32f7xx_hal_spi.o \
		 ${COMPILER}/stm32f7xx_hal_flash.o \
		 ${COMPILER}/stm32f7xx_hal_flash_ex.o \
		 ${COMPILER}/stm32f7xx_ll_usb.o \
		 ${COMPILER}/stm32f7xx_ll_fmc.o
##############################################################################
#
#   The flags passed to the C compiler.
#
##############################################################################
CFLAGS_DEF+=

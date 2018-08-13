##############################################################################
#
#   Objects Files
#
##############################################################################

OBJECTS+= ${COMPILER}/audio_codec.o \
		  ${COMPILER}/DOA.o \
		  ${COMPILER}/DSP.o \
		  ${COMPILER}/sta321mp.o \
		  ${COMPILER}/usbh_diskio.o \
		  ${COMPILER}/waverecorder.o \
		  ${COMPILER}/DelayEstimation.o \
		  ${COMPILER}/usbd_audio_if.o \
		  ${COMPILER}/usbd_desc.o \
		  ${COMPILER}/usbh_conf.o \
		  ${COMPILER}/usbd_conf_f4.o \


##############################################################################
#
#   The flags passed to the C compiler.
#
##############################################################################
CFLAGS_DEF+=
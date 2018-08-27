#ifndef __COMMAND_H
#define __COMMAND_H

//Define Object
//Value range from 0x00 - 0x1F
#define LED_RING 			0x00
#define MIC_ARRAY 			0x01
#define CYPRESS_BUTTON		0x02

//Define Command 
//Value range from 0x20 to 0xFF
#define VOLUME_UP			0x20
#define VOLUME_DOWN			0x21
#define VOLUME_MUTE			0x22
#define WAKE_WORD_START		0x23
#define WAKE_WORD_STOP		0x24

#define LED_DIMMING			0x30
#define LED_PATTERN			0x31
#define LED_WAITING			0x32
#define LED_WW_START		0x33
#define LED_WW_STOP			0x34

#endif /* __COMMAND_H */
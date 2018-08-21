#ifndef __COMMAND_H
#define __COMMAND_H

//Define Object
//Value range from 0x00 - 0x1F
#define LED_RING 		0x00
#define MIC_ARRAY 		0x01
#define CYPRESS_BT1		0x02
#define CYPRESS_BT2		0x03
#define CYPRESS_BT3 	0x04
#define CYPRESS_BT4 	0x05
//Define Command 
//Value range from 0x20 to 0xFF
#define VOLUME_UP		0x20
#define VOLUME_DOWN		0x21
#define VOLUME_MUTE		0x22
#define WAKE_WORD		0x23
#endif
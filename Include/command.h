#ifndef __COMMAND_H
#define __COMMAND_H

//Define Object
//Value range from 0x00 - 0x1F
#define LED_RING 			0x00
#define MIC_ARRAY 			0x01
#define CYPRESS_BUTTON		0x02
#define USER_EVENT			0x03

//Define Command 
//Value range from 0x20 to 0xFF
#define VOLUME_UP			0x20
#define VOLUME_DOWN			0x21
#define VOLUME_MUTE			0x22
#define BT_WAKEWORD_START	0x23
#define WAKE_WORD_STOP		0x24
#define VOLUME_UNMUTE		0x25
#define MICROPHONE_MUTE		0x26
#define MICROPHONE_UNMUTE	0x27

#define LED_DIMMING			0x30
#define LED_PATTERN			0x31
#define LED_WAITING			0x32
#define LED_WW_START		0x33
#define LED_WW_STOP			0x34

#define WIFI_CONNECTED		0x40
#define WIFI_DISCONNECTED	0x41
#define RECORD_ERROR		0x42
#define BLE_ON				0x43
#define BLE_OFF				0x44
#endif /* __COMMAND_H */
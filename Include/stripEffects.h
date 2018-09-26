#ifndef __STRIPEFFECTS
#define __STRIPEFFECTS

#include "main.h"
#include "ws281x.h"
#define CLEAR_ALL_LEDS()		setWHOLEcolor(0, 0, 0)
#define HEARTBEAT_STEPS			16

void stripEffect_CircularRing(uint32_t interval, uint8_t red, uint8_t green,
		uint8_t blue);
void stripEffect_HeartBeat(uint32_t interval, uint8_t red, uint8_t green,
		uint8_t blue);
void stripEffect_ColorWheel(uint32_t interval);
void stripEffect_AllColors(uint32_t interval);
void stripEffect_PatternMove(uint32_t interval, uint32_t parts, uint8_t red,
		uint8_t green, uint8_t blue);
void stripEffect_FullEmpty(uint32_t interval, uint8_t red, uint8_t green,
		uint8_t blue);
void stripEffect_FromTo(uint32_t interval, uint32_t steps, uint8_t redA,
		uint8_t greenA, uint8_t blueA, uint8_t redB, uint8_t greenB,
		uint8_t blueB);
void stripEffect_AlternateColors(uint32_t interval, uint32_t steps,
		uint8_t redA, uint8_t greenA, uint8_t blueA, uint8_t redB,
		uint8_t greenB, uint8_t blueB);

#endif

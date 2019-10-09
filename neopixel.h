/*
 * NeoPixel WS2812B Driver for nRF52 series.
 *  (c) 2019, Aquamarine Networks.
 */

#ifndef _NEOPIXEL_H_
#define _NEOPIXEL_H_

#include "nrf_drv_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NEOPIXEL_GRB(green,red,blue) ( ((uint8_t)(blue)) | (((uint8_t)(red)) << 8) | (((uint8_t)(green)) << 16))

typedef  void (*neopixel_handler_t)(void);

void neopixel_init(uint8_t pin, neopixel_handler_t handler);
void neopixel_uninit(void);
void neopixel_write(uint32_t *colors, uint32_t chain);
uint32_t neopixel_HSVtoGRB(double dH, double dS, double dV);

#ifdef __cplusplus
}
#endif

#endif //_NEOPIXEL_H_

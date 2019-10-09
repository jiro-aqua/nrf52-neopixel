/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"

#include "neopixel.h"

APP_TIMER_DEF(m_timer_0);

static void neopixel_handler(void)
{
  // todo: finished writing neopixel 
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    app_error_save_and_stop(id, pc, info);
}

static uint8_t h = 0;
static uint8_t s = 100;
static uint8_t v = 0;
static uint8_t sign = 1;

static uint32_t gradient(uint32_t t) {
  return (uint32_t)(49.0 * cos(M_PI / 100.0 * t + M_PI) + 51.0);
}

static void timer_handle(void *p_context) {
  UNUSED_PARAMETER(p_context);

  if (sign == 1) {
    v++;
    if (v == 100){
      sign = 0;
    }
  } else {
    v--;
    if (v == 0){
      sign = 1;
      h = rand() % 360;
    }
  }

  uint32_t color = neopixel_HSVtoGRB(h, s, gradient(v));
  neopixel_write(&color, 1);
}

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("NeoPixel example started.");

    lfclk_request();

    uint32_t ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    ret = app_timer_create(&m_timer_0, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_timer_0, APP_TIMER_TICKS(10), NULL);
    APP_ERROR_CHECK(ret);

    uint8_t neoPixelPin = NRF_GPIO_PIN_MAP(0,16);

    neopixel_init(neoPixelPin, neopixel_handler);

    uint32_t color = 0x000000;  // 0x00GGRRBB
    neopixel_write(&color,1);

    for (;;)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}


/** @} */
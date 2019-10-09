/*
 * NeoPixel WS2812B Driver for nRF52 series.
 *  (c) 2019, Aquamarine Networks.
 */


#include "neopixel.h"

#include "math.h"

// change this for your settings
#define NEOPIXEL_MAX_CHAINS 10
#define NEOPIXEL_INSTANCE 0

#define NEOPIXEL_BYTES 24
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(NEOPIXEL_INSTANCE);
static nrf_pwm_values_common_t pwm_sequence_values[NEOPIXEL_MAX_CHAINS * NEOPIXEL_BYTES + 1];
static neopixel_handler_t m_handler = NULL;

enum {
  CLOCK = NRF_PWM_CLK_16MHz,
  TOP = 20,
  DUTY0 = 6,
  DUTY1 = 13,
};

static void pwm_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
      if ( m_handler != NULL ){
        m_handler();
      }
    }
}


void neopixel_init(uint8_t pin, neopixel_handler_t handler) {
  m_handler = handler;
  nrf_drv_pwm_config_t const config0 =
      {
          .output_pins =
              {
                  pin,                      // channel 0
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 1
                  NRF_DRV_PWM_PIN_NOT_USED, // channel 2
                  NRF_DRV_PWM_PIN_NOT_USED  // channel 3
              },
          .irq_priority = APP_IRQ_PRIORITY_LOWEST,
          .base_clock = CLOCK,
          .count_mode = NRF_PWM_MODE_UP,
          .top_value = TOP,
          .load_mode = NRF_PWM_LOAD_COMMON,
          .step_mode = NRF_PWM_STEP_AUTO};
  APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, pwm_handler));
}

void neopixel_uninit(void) {
  nrf_drv_pwm_uninit(&m_pwm0);
}


void neopixel_write(uint32_t *colors, uint32_t chain) {
  if (chain > NEOPIXEL_MAX_CHAINS) {
    chain = NEOPIXEL_MAX_CHAINS - 1;
  }

  nrf_pwm_values_common_t *ptr = pwm_sequence_values;
  for (uint32_t led = 0; led < chain; led++) {
    uint32_t color = colors[led];
    for (uint8_t i = 0; i < NEOPIXEL_BYTES; ++i) {
      uint16_t value = 0;
      if ((color & 0x800000) == 0) {
        value = DUTY0;
      } else {
        value = DUTY1;
      }
      *ptr++ = value | 0x8000;
      color <<= 1;
    }
  }
  *ptr++ = 0x8000;

  nrf_pwm_sequence_t const seq0 =
      {
          .values.p_common = pwm_sequence_values,
          .length = NEOPIXEL_BYTES * chain + 1,
          .repeats = 0,
          .end_delay = 0};

  (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1, NRF_DRV_PWM_FLAG_STOP);
}


static double Min(double a, double b, double c) {
  if (a <= b && a <= c)
    return a;
  if (b <= a && b <= c)
    return b;
  return c;
}

static double Max(double a, double b, double c) {
  if (a >= b && a >= c)
    return a;
  if (b >= a && b >= c)
    return b;
  return c;
}


uint32_t neopixel_HSVtoGRB(double dH, double dS, double dV) {
  dS /= 100.0;
  dV /= 100.0;

  uint32_t grb = 0;
  if (dS == 0.0) {
    uint8_t v = (uint8_t)(dV * 255);
    return NEOPIXEL_GRB(v, v, v);
  }

  double f;
  double p;
  double q;
  double t;
  int nHi;

  nHi = (int)(dH / 60) % 6;
  if (nHi < 0)
    nHi *= -1;

  f = dH / 60 - nHi;
  p = dV * (1 - dS);
  q = dV * (1 - f * dS);
  t = dV * (1 - (1 - f) * dS);

  dV = dV * 255;
  p = p * 255;
  q = q * 255;
  t = t * 255;

  if (dV > 255)
    dV = 255;
  if (t > 255)
    t = 255;
  if (p > 255)
    p = 255;
  if (q > 255)
    q = 255;

  if (nHi == 0) {
    return NEOPIXEL_GRB(t, dV, p);
  }
  if (nHi == 1) {
    return NEOPIXEL_GRB(dV, q, p);
  }
  if (nHi == 2) {
    return NEOPIXEL_GRB(dV, p, t);
  }
  if (nHi == 3) {
    return NEOPIXEL_GRB(q, p, dV);
  }
  if (nHi == 4) {
    return NEOPIXEL_GRB(p, t, dV);
  }
  if (nHi == 5) {
    return NEOPIXEL_GRB(p, dV, q);
  }
  return 0;
}


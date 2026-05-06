#include "nlms_square_protect.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf_local(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

void nlms_update_notch_init(nlms_update_notch_t *f, float sample_rate_hz,
                            float notch_r) {
  memset(f, 0, sizeof(*f));
  f->sample_rate_hz = sample_rate_hz;
  f->r = clampf_local(notch_r, 0.90f, 0.9999f);
  f->b0 = 1.0f;
  f->b1 = 0.0f;
  f->b2 = 0.0f;
  f->a1 = 0.0f;
  f->a2 = 0.0f;
  f->valid = 0;
}

void nlms_update_notch_reset(nlms_update_notch_t *f) {
  f->x1 = 0.0f;
  f->x2 = 0.0f;
  f->y1 = 0.0f;
  f->y2 = 0.0f;
}

uint8_t nlms_update_notch_set_freq(nlms_update_notch_t *f,
                                   float protect_freq_hz) {
  if ((f->sample_rate_hz <= 0.0f) ||
      (protect_freq_hz < NLMS_SQUARE_PROTECT_MIN_FREQ_HZ) ||
      (protect_freq_hz > NLMS_SQUARE_PROTECT_MAX_FREQ_HZ) ||
      (protect_freq_hz > 0.45f * f->sample_rate_hz)) {
    f->valid = 0;
    return 0;
  }

  const float w0 = 2.0f * (float)M_PI * protect_freq_hz / f->sample_rate_hz;
  const float c = cosf(w0);
  const float r = clampf_local(f->r, 0.90f, 0.9999f);

  f->b0 = 1.0f;
  f->b1 = -2.0f * c;
  f->b2 = 1.0f;
  f->a1 = -2.0f * r * c;
  f->a2 = r * r;
  f->notch_freq_hz = protect_freq_hz;
  f->valid = 1;
  return 1;
}

float nlms_update_notch_process(nlms_update_notch_t *f, float e_raw) {
  if (f->valid == 0U) {
    return e_raw;
  }

  const float y0 = f->b0 * e_raw + f->b1 * f->x1 + f->b2 * f->x2 -
                   f->a1 * f->y1 - f->a2 * f->y2;

  f->x2 = f->x1;
  f->x1 = e_raw;
  f->y2 = f->y1;
  f->y1 = y0;

  return y0;
}

void nlms_square_protect_default_config(nlms_square_protect_config_t *cfg,
                                        float base_mu, float square_mu) {
  cfg->base_mu = base_mu;
  cfg->square_mu = square_mu;
  cfg->min_square_ref_freq_hz = NLMS_SQUARE_REF_MIN_FREQ_HZ;
  cfg->max_square_ref_freq_hz = NLMS_SQUARE_REF_MAX_FREQ_HZ;
  cfg->min_protect_freq_hz = NLMS_SQUARE_PROTECT_MIN_FREQ_HZ;
  cfg->max_protect_freq_hz = NLMS_SQUARE_PROTECT_MAX_FREQ_HZ;
  cfg->min_protect_confidence = 0.50f;
  cfg->notch_r = NLMS_SQUARE_PROTECT_DEFAULT_R;
}

uint8_t nlms_square_protect_should_enable(
    const nlms_square_protect_config_t *cfg, uint8_t ref_is_square,
    float ref_square_freq_hz, float protect_freq_hz,
    float protect_confidence) {
  if (ref_is_square == 0U) {
    return 0;
  }

  if ((ref_square_freq_hz < cfg->min_square_ref_freq_hz) ||
      (ref_square_freq_hz > cfg->max_square_ref_freq_hz)) {
    return 0;
  }

  if ((protect_freq_hz < cfg->min_protect_freq_hz) ||
      (protect_freq_hz > cfg->max_protect_freq_hz)) {
    return 0;
  }

  if (protect_confidence < cfg->min_protect_confidence) {
    return 0;
  }

  return 1;
}

float nlms_square_protect_mu(const nlms_square_protect_config_t *cfg,
                             uint8_t protect_enabled) {
  return protect_enabled ? cfg->square_mu : cfg->base_mu;
}

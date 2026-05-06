#ifndef NLMS_SQUARE_PROTECT_H
#define NLMS_SQUARE_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef NLMS_SQUARE_PROTECT_DEFAULT_R
#define NLMS_SQUARE_PROTECT_DEFAULT_R (0.995f)
#endif

#ifndef NLMS_SQUARE_PROTECT_MIN_FREQ_HZ
#define NLMS_SQUARE_PROTECT_MIN_FREQ_HZ (20.0f)
#endif

#ifndef NLMS_SQUARE_PROTECT_MAX_FREQ_HZ
#define NLMS_SQUARE_PROTECT_MAX_FREQ_HZ (5000.0f)
#endif

#ifndef NLMS_SQUARE_REF_MIN_FREQ_HZ
#define NLMS_SQUARE_REF_MIN_FREQ_HZ (20.0f)
#endif

#ifndef NLMS_SQUARE_REF_MAX_FREQ_HZ
#define NLMS_SQUARE_REF_MAX_FREQ_HZ (600.0f)
#endif

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;

  float x1;
  float x2;
  float y1;
  float y2;

  float sample_rate_hz;
  float notch_freq_hz;
  float r;
  uint8_t valid;
} nlms_update_notch_t;

typedef struct {
  float base_mu;
  float square_mu;
  float min_square_ref_freq_hz;
  float max_square_ref_freq_hz;
  float min_protect_freq_hz;
  float max_protect_freq_hz;
  float min_protect_confidence;
  float notch_r;
} nlms_square_protect_config_t;

void nlms_update_notch_init(nlms_update_notch_t *f, float sample_rate_hz,
                            float notch_r);

void nlms_update_notch_reset(nlms_update_notch_t *f);

uint8_t nlms_update_notch_set_freq(nlms_update_notch_t *f,
                                   float protect_freq_hz);

float nlms_update_notch_process(nlms_update_notch_t *f, float e_raw);

void nlms_square_protect_default_config(nlms_square_protect_config_t *cfg,
                                        float base_mu, float square_mu);

uint8_t nlms_square_protect_should_enable(
    const nlms_square_protect_config_t *cfg, uint8_t ref_is_square,
    float ref_square_freq_hz, float protect_freq_hz,
    float protect_confidence);

float nlms_square_protect_mu(const nlms_square_protect_config_t *cfg,
                             uint8_t protect_enabled);

#ifdef __cplusplus
}
#endif

#endif /* NLMS_SQUARE_PROTECT_H */

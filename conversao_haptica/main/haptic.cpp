/*
 * haptic.cpp — CONVERSÃO DO ÁUDIO PARA VIBRAÇÃO  (ESP-IDF v5.x, C++)
 */
#include "haptic.hpp"
#include <math.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "haptic";

/* Pinos dos transistores (base via resistor). Ajuste conforme a sua montagem. */
#define HAPTIC_PIN_PALM        GPIO_NUM_4
#define HAPTIC_PIN_FINGERS     GPIO_NUM_5
#define HAPTIC_PWM_FREQ_HZ     20000      /* acima do audível; o ERM faz a média */

static inline float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

/* ===================================================================== */
/*                       LÓGICA: voz -> intensidades                      */
/* ===================================================================== */

HapticMapper::HapticMapper(const HapticParams &params) : p_(params) {}

void HapticMapper::advance_timers(float dt)
{
    if (pulse_t_      > 0.f) pulse_t_      -= dt;
    if (refractory_t_ > 0.f) refractory_t_ -= dt;
    if (kick_t_       > 0.f) kick_t_       -= dt;
}

HapticCommand HapticMapper::update(const VoiceFeatures &f, float dt)
{
    advance_timers(dt);

    /* ---------- 1) Nível em dB -> 0..1 (relativo ao ruído e ao pico da fala) ---------- */
    float norm = 0.f;
    if (f.rms > 1e-6f) {
        float level_db = 20.f * log10f(f.rms);
        if (!level_initialized_) {
            noise_db_ = level_db;
            peak_db_  = level_db + 30.f;
            level_initialized_ = true;
        }
        /* piso de ruído: desce rápido, sobe devagar (nunca passa do nível atual) */
        if (level_db < noise_db_) noise_db_ = level_db;
        else noise_db_ = fminf(noise_db_ + p_.noise_rise_db_per_s * dt, level_db);

        /* referência de pico: sobe rápido, cai devagar, mantendo faixa dinâmica mínima */
        peak_db_ = fmaxf(level_db, peak_db_ - p_.peak_decay_db_per_s * dt);
        peak_db_ = fmaxf(peak_db_, noise_db_ + p_.gate_db + 12.f);

        float span = peak_db_ - noise_db_ - p_.gate_db;
        norm = clamp01((level_db - noise_db_ - p_.gate_db) / span);
    }
    /* rms ~ 0 (sem dados/silêncio): norm = 0, sem mexer no piso de ruído */

    /* envelope da palma (ataque rápido, soltura lenta) e energia para sílabas */
    float tau = (norm > env_) ? p_.attack_s : p_.release_s;
    env_     += (1.f - expf(-dt / tau)) * (norm - env_);
    syl_env_ += (1.f - expf(-dt / p_.syl_smooth_s)) * (norm - syl_env_);
    voicing_  = clamp01(f.voicing);

    /* ---------- 2) F0 em semitons, relativo à média recente do falante ---------- */
    float f0_delta = 0.f;
    if (f.voicing > 0.5f && f.f0_hz > 50.f) {
        float st = 12.f * log2f(f.f0_hz / 100.f);
        if (!f0_base_valid_) { f0_base_st_ = st; f0_base_valid_ = true; }
        f0_delta = fmaxf(0.f, st - f0_base_st_);
        f0_base_st_ += (1.f - expf(-dt / p_.f0_base_tau_s)) * (st - f0_base_st_);
    }

    /* ---------- 3) Detecção de sílaba e proeminência (tonicidade) ---------- */
    if (syl_env_ > p_.syl_onset) {
        syl_dur_ += dt;
        if (syl_env_ > cand_peak_) cand_peak_ = syl_env_;       /* acompanha o pico  */
        if (f0_delta > cand_f0_delta_) cand_f0_delta_ = f0_delta;
    } else {
        syl_dur_ = 0.f;                                          /* fora de sílaba    */
        cand_peak_ = syl_env_;
        cand_f0_delta_ = 0.f;
    }

    /* pico confirmado: a energia caiu 'syl_drop' abaixo do máximo */
    if (cand_peak_ > p_.syl_min_peak && syl_env_ < cand_peak_ - p_.syl_drop) {
        float score = p_.w_energy   * clamp01(cand_peak_)
                    + p_.w_f0       * clamp01(cand_f0_delta_ / p_.f0_full_semitones)
                    + p_.w_duration * clamp01(syl_dur_ / p_.dur_full_s);

        if (score >= p_.tonic_threshold && pulse_t_ <= 0.f && refractory_t_ <= 0.f) {
            pulse_t_      = p_.finger_pulse_s;
            refractory_t_ = p_.finger_pulse_s + p_.finger_refractory_s;
            ESP_LOGD(TAG, "tônica: pico=%.2f dF0=%.1fst dur=%.0fms score=%.2f",
                     cand_peak_, cand_f0_delta_, syl_dur_ * 1000.f, score);
        }
        cand_peak_ = syl_env_;            /* recomeça a busca por um novo pico */
        cand_f0_delta_ = 0.f;
        syl_dur_ = 0.f;
    }

    return output();
}

HapticCommand HapticMapper::tick(float dt)
{
    advance_timers(dt);
    return output();
}

HapticCommand HapticMapper::output()
{
    /* liga/desliga com histerese (evita tremer no limiar) */
    if (!palm_on_ && env_ > 0.04f)      { palm_on_ = true; kick_t_ = p_.kick_s; }
    else if (palm_on_ && env_ < 0.02f)  { palm_on_ = false; }

    HapticCommand cmd = {0.f, 0.f};

    if (palm_on_) {
        float shape = powf(env_, p_.gamma) * (0.6f + 0.4f * voicing_);
        float d = p_.palm_min_duty + (p_.palm_max_duty - p_.palm_min_duty) * shape;
        if (kick_t_ > 0.f) d = 1.f;                         /* kick-start */
        if (pulse_t_ > 0.f) d = fmaxf(d * p_.palm_duck, p_.palm_min_duty);
        cmd.palm = clamp01(d);
    }
    if (pulse_t_ > 0.f) cmd.fingers = p_.finger_duty;
    return cmd;
}

/* ===================================================================== */
/*                         HARDWARE: PWM + TASK                           */
/* ===================================================================== */

#define CH_PALM     LEDC_CHANNEL_0
#define CH_FINGERS  LEDC_CHANNEL_1

static QueueHandle_t features_queue = NULL;

static void pwm_init(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.timer_num       = LEDC_TIMER_0;
    timer.freq_hz         = HAPTIC_PWM_FREQ_HZ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const struct { gpio_num_t pin; ledc_channel_t ch; } outputs[] = {
        { HAPTIC_PIN_PALM,    CH_PALM    },
        { HAPTIC_PIN_FINGERS, CH_FINGERS },
    };
    for (const auto &o : outputs) {
        ledc_channel_config_t ch = {};
        ch.gpio_num   = o.pin;
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel    = o.ch;
        ch.timer_sel  = LEDC_TIMER_0;
        ch.duty       = 0;
        ch.hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
}

static void pwm_set(ledc_channel_t ch, float duty01)
{
    uint32_t value = (uint32_t)(clamp01(duty01) * 255.f + 0.5f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void haptic_task(void *parameters)
{
    HapticMapper mapper;
    int64_t last_us = esp_timer_get_time();
    int64_t last_features_us = last_us;

    while (1) {
        VoiceFeatures features;
        bool got = (xQueueReceive(features_queue, &features, pdMS_TO_TICKS(10)) == pdTRUE);

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.100f) dt = 0.100f;

        HapticCommand cmd;
        if (got) {
            last_features_us = now_us;
            cmd = mapper.update(features, dt);
        } else if (now_us - last_features_us > 200000) {   /* 200 ms sem dados = silêncio */
            VoiceFeatures silence = {0.f, 0.f, 0.f};
            cmd = mapper.update(silence, dt);
        } else {
            cmd = mapper.tick(dt);
        }

        pwm_set(CH_PALM,    cmd.palm);
        pwm_set(CH_FINGERS, cmd.fingers);
    }
}

void haptic_init(void)
{
    pwm_init();
    features_queue = xQueueCreate(1, sizeof(VoiceFeatures));   /* só o quadro mais recente */
    xTaskCreatePinnedToCore(haptic_task, "haptic_task", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "haptic pronto: palma=GPIO%d dedos=GPIO%d", (int)HAPTIC_PIN_PALM, (int)HAPTIC_PIN_FINGERS);
}

void haptic_submit_features(const VoiceFeatures &features)
{
    if (features_queue) xQueueOverwrite(features_queue, &features);
}

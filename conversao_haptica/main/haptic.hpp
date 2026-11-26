/*
 * haptic.hpp — CONVERSÃO DO ÁUDIO PARA VIBRAÇÃO (CORE 1)
 *
 * Recebe características da voz (nível, F0, vozeamento) e gera dois comandos PWM:
 *   - PALMA  : fluxo geral da fala (intensidade acompanha a energia da voz)
 *   - DEDOS  : pulso curto quando uma sílaba é considerada tônica
 *
 * Entrada: VoiceFeatures (vem do pYIN da Vitória + nível RMS do áudio filtrado)
 * Saída  : PWM (LEDC) nos transistores dos 2 motores ERM
 */
#pragma once
#include <stdint.h>

/* Características de UM quadro de voz (áudio filtrado 75-500 Hz) */
struct VoiceFeatures {
    float f0_hz;      /* frequência fundamental em Hz (0 se não vozeado)          */
    float voicing;    /* 0..1  confiança de vozeamento (probabilidade do pYIN)    */
    float rms;        /* 0..1  RMS do quadro, normalizado pelo fundo de escala    */
};

/* Comando de saída: ciclo de trabalho (duty) 0..1 de cada motor */
struct HapticCommand {
    float palm;
    float fingers;
};

/* Todos os parâmetros ajustáveis ficam aqui (o dicionário de vibrações
 * "será definido experimentalmente" — mexa só neste bloco). */
struct HapticParams {
    /* ---- nível -> intensidade da palma ---- */
    float gate_db             = 6.0f;    /* acima do piso de ruído para abrir o gate    */
    float noise_rise_db_per_s = 0.5f;    /* velocidade com que o piso de ruído sobe     */
    float peak_decay_db_per_s = 1.0f;    /* velocidade com que a referência de pico cai */
    float attack_s            = 0.020f;  /* subida do envelope                          */
    float release_s           = 0.080f;  /* descida do envelope                         */
    float gamma               = 0.7f;    /* <1 realça níveis baixos (tato é pouco sensível) */
    float palm_min_duty       = 0.35f;   /* abaixo disso o ERM não gira                 */
    float palm_max_duty       = 1.00f;
    float kick_s              = 0.030f;  /* pulso de 100% para vencer a inércia do ERM  */

    /* ---- sílaba tônica ---- */
    float syl_smooth_s        = 0.030f;  /* suavização da energia p/ achar picos        */
    float syl_onset           = 0.15f;   /* energia mínima p/ considerar "dentro" de sílaba */
    float syl_min_peak        = 0.25f;   /* pico mínimo para ser sílaba                 */
    float syl_drop            = 0.15f;   /* queda após o pico que confirma a sílaba     */
    float w_energy            = 0.5f;    /* pesos da proeminência (somam 1)             */
    float w_f0                = 0.3f;
    float w_duration          = 0.2f;
    float f0_full_semitones   = 4.0f;    /* subida de F0 que vale nota máxima           */
    float dur_full_s          = 0.25f;   /* duração que vale nota máxima                */
    float f0_base_tau_s       = 1.5f;    /* janela da média de F0 do falante            */
    float tonic_threshold     = 0.60f;   /* proeminência mínima para ser tônica         */

    /* ---- dedos ---- */
    float finger_pulse_s      = 0.120f;
    float finger_refractory_s = 0.150f;
    float finger_duty         = 1.0f;
    float palm_duck           = 0.5f;    /* reduz a palma durante o pulso (separa os canais) */
};

/* Lógica pura (sem hardware): dá para testar no PC com valores fictícios. */
class HapticMapper {
public:
    explicit HapticMapper(const HapticParams &params = HapticParams());

    /* Chamar a cada quadro novo de características. dt_s = tempo desde a última chamada. */
    HapticCommand update(const VoiceFeatures &features, float dt_s);

    /* Chamar entre quadros (só avança pulsos/kick e mantém a saída). */
    HapticCommand tick(float dt_s);

private:
    HapticCommand output();
    void advance_timers(float dt_s);

    HapticParams p_;

    /* nível */
    bool  level_initialized_ = false;
    float noise_db_ = -90.f, peak_db_ = -40.f;
    float env_ = 0.f, syl_env_ = 0.f, voicing_ = 0.f;
    bool  palm_on_ = false;

    /* F0 */
    bool  f0_base_valid_ = false;
    float f0_base_st_ = 0.f;

    /* sílaba */
    float cand_peak_ = 0.f, cand_f0_delta_ = 0.f, syl_dur_ = 0.f;

    /* temporizadores (s) */
    float pulse_t_ = 0.f, refractory_t_ = 0.f, kick_t_ = 0.f;
};

/* Camada de hardware: cria o PWM (LEDC) e a task no CORE 1. */
void haptic_init(void);

/* Envia o quadro mais recente (não bloqueia; sobrescreve o anterior). */
void haptic_submit_features(const VoiceFeatures &features);

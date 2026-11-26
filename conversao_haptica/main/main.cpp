#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "haptic.hpp"

extern "C" void app_main(void)
{
    haptic_init();

    while (1) {
        /* "sílaba" de ~300 ms: energia 0 -> 1 -> 0, F0 sobe de 120 para ~150 Hz */
        for (int i = 0; i < 30; i++) {
            float x = sinf((float)M_PI * i / 30.0f);
            VoiceFeatures f;
            f.rms     = 0.002f + 0.1f * x;
            f.f0_hz   = 120.0f + 30.0f * x;
            f.voicing = 1.0f;
            haptic_submit_features(f);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        /* silêncio de ~600 ms (nível de ruído, para o piso não sair do lugar) */
        for (int i = 0; i < 60; i++) {
            VoiceFeatures f = {0.0f, 0.0f, 0.002f};
            haptic_submit_features(f);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
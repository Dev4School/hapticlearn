#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "INMP441_TEST";

// Pinagem definida para o ESP32-S3
#define I2S_WS_IO       GPIO_NUM_13
#define I2S_SCK_IO      GPIO_NUM_11
#define I2S_SD_IO       GPIO_NUM_16

#define SAMPLE_RATE     16000
#define BUFFER_SAMPLES  512

static i2s_chan_handle_t rx_handle = NULL;

static void i2s_init_std_rx(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        // INMP441 envia 24 bits alinhados à esquerda dentro de uma palavra de 32 bits (Philips)
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    // Garante leitura do canal Esquerdo (pino L/R do INMP441 ligado ao GND)
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2S para o INMP441...");
    i2s_init_std_rx();
    ESP_LOGI(TAG, "Leitura iniciada. Fale perto do microfone.");

    int32_t *raw_buffer = (int32_t *)malloc(BUFFER_SAMPLES * sizeof(int32_t));
    if (raw_buffer == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar memoria para o buffer");
        return;
    }

    size_t bytes_read = 0;

    while (1) {
        esp_err_t ret = i2s_channel_read(rx_handle, raw_buffer, BUFFER_SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);
        
        if (ret == ESP_OK && bytes_read > 0) {
            int samples_read = bytes_read / sizeof(int32_t);
            double sum_squares = 0.0;

            for (int i = 0; i < samples_read; i++) {
                // Desloca 8 bits para a direita para extrair os 24 bits úteis com sinal
                int32_t sample_24bit = raw_buffer[i] >> 8;
                sum_squares += (double)sample_24bit * (double)sample_24bit;
            }

            double rms = sqrt(sum_squares / samples_read);

            // Monta uma barra visual simples no terminal
            int bars = (int)(rms / 15000.0);
            if (bars > 40) bars = 40;

            char meter[45];
            for (int b = 0; b < 40; b++) {
                meter[b] = (b < bars) ? '#' : ' ';
            }
            meter[40] = '\0';

            printf("RMS: %10.0f | [%s]\n", rms, meter);
        } else {
            ESP_LOGW(TAG, "Erro na leitura I2S: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    free(raw_buffer);
}

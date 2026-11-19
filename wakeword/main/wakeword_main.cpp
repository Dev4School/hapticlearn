#include <cstdlib>
#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"

extern "C" {
#include "model_path.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
}

static const char *TAG = "WAKEWORD";

#define I2S_BCLK    GPIO_NUM_4
#define I2S_WS      GPIO_NUM_5
#define I2S_DIN     GPIO_NUM_6
#define SAMPLE_RATE 16000

static i2s_chan_handle_t rx_chan;

static void i2s_init()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN,
                        .invert_flags = {
                .mclk_inv = 0,
                .bclk_inv = 0,
                .ws_inv   = 0,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  // L/R no GND = canal esquerdo

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static void wakeword_task(void *arg)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == nullptr) {
        ESP_LOGE(TAG, "Falha ao carregar os modelos da particao");
        vTaskDelete(NULL);
        return;
    }

    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    if (wn_name == nullptr) {
        ESP_LOGE(TAG, "Nenhum modelo de wakeword encontrado");
        esp_srmodel_deinit(models);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Modelo: %s", wn_name);

    const esp_wn_iface_t *wakenet = esp_wn_handle_from_name(wn_name);
    if (wakenet == nullptr) {
        ESP_LOGE(TAG, "Interface WakeNet nao encontrada para o modelo %s", wn_name);
        esp_srmodel_deinit(models);
        vTaskDelete(NULL);
        return;
    }

    model_iface_data_t *model_data = wakenet->create(wn_name, DET_MODE_95);
    if (model_data == nullptr) {
        ESP_LOGE(TAG, "Falha ao criar a instancia do modelo WakeNet");
        esp_srmodel_deinit(models);
        vTaskDelete(NULL);
        return;
    }

    const int chunk = wakenet->get_samp_chunksize(model_data);
    if (chunk <= 0) {
        ESP_LOGE(TAG, "Tamanho de bloco WakeNet invalido: %d", chunk);
        wakenet->destroy(model_data);
        esp_srmodel_deinit(models);
        vTaskDelete(NULL);
        return;
    }

    int32_t *samples = static_cast<int32_t *>(malloc(chunk * sizeof(int32_t)));
    if (samples == nullptr) {
        ESP_LOGE(TAG, "Sem memoria para o buffer de audio");
        wakenet->destroy(model_data);
        esp_srmodel_deinit(models);
        vTaskDelete(NULL);
        return;
    }
    int16_t *pcm = reinterpret_cast<int16_t *>(samples);

    ESP_LOGI(TAG, "Escutando... (%d amostras por bloco)", chunk);

    while (true) {
        size_t bytes_read = 0;
        i2s_channel_read(rx_chan, samples, chunk * sizeof(int32_t), &bytes_read, portMAX_DELAY);

        if (bytes_read != chunk * sizeof(int32_t)) {
            continue;
        }

        for (int i = 0; i < chunk; i++) {
            int32_t s = samples[i] >> 14;
            if (s > 32767)  s = 32767;
            if (s < -32768) s = -32768;
            int16_t sample = static_cast<int16_t>(s);
            memcpy(&pcm[i], &sample, sizeof(sample));
        }

        int word = wakenet->detect(model_data, pcm);
        if (word > 0) {
            ESP_LOGI(TAG, ">>> WAKEWORD DETECTADA: %s <<<",
                     wakenet->get_word_name(model_data, word));
        }
    }
}

extern "C" void app_main(void)
{
    i2s_init();
    xTaskCreatePinnedToCore(wakeword_task, "wakeword", 8 * 1024, NULL, 5, NULL, 1);
}

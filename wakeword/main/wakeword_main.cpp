#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"

extern "C" {
#include "model_path.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
}

static const char *TAG = "WAKEWORD";
static const char *TAG_MAIN = "MAIN";

// ---- Pinos e áudio ----
#define I2S_BCLK    GPIO_NUM_4
#define I2S_WS      GPIO_NUM_5
#define I2S_DIN     GPIO_NUM_6
#define SAMPLE_RATE 16000

// ---- Configuração do silêncio ----
#define SILENCE_TIMEOUT_MS   4000    // tempo sem fala para desativar o mic
#define MAX_ACTIVE_MS        60000   // segurança: duração máxima de uma sessão ativa
#define VOICE_RMS_THRESHOLD  500.0f  // AJUSTAR com o seu mic (veja o log de RMS)
#define DEBUG_RMS            1       // 1 = imprime o RMS no modo ativo para calibrar

static i2s_chan_handle_t rx_chan;

// Estado do microfone: true = captando fala, false = só esperando a wake word.
// Pode ser lido por outras tarefas (Core 1, envio Wi-Fi etc.)
volatile bool g_mic_active = false;

// Protótipos das tarefas
static void wakeword_task(void *arg);
void TarefaCore1(void *pvParameters);

static inline int64_t now_ms()
{
    return esp_timer_get_time() / 1000;
}

static float chunk_rms(const int16_t *pcm, int n)
{
    int64_t acc = 0;
    for (int i = 0; i < n; i++) {
        acc += (int32_t)pcm[i] * (int32_t)pcm[i];
    }
    return sqrtf((float)acc / (float)n);
}

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

// Tarefa da WAKEWORD + controle de silêncio (roda no Core 0)
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

    int64_t active_start = 0;
    int64_t last_voice   = 0;
    int     dbg_counter  = 0;

    while (true) {
        size_t bytes_read = 0;
        i2s_channel_read(rx_chan, samples, chunk * sizeof(int32_t), &bytes_read, portMAX_DELAY);

        if (bytes_read != chunk * sizeof(int32_t)) {
            continue;
        }

        // Converte 32 bits (INMP441) para 16 bits
        for (int i = 0; i < chunk; i++) {
            int32_t s = samples[i] >> 14;
            if (s > 32767)  s = 32767;
            if (s < -32768) s = -32768;
            int16_t sample = static_cast<int16_t>(s);
            memcpy(&pcm[i], &sample, sizeof(sample));
        }

        if (!g_mic_active) {
            // ===== MODO ESPERA: só procura a wake word =====
            int word = wakenet->detect(model_data, pcm);
            if (word > 0) {
                ESP_LOGI(TAG, ">>> WAKEWORD DETECTADA: %s <<<",
                         wakenet->get_word_name(model_data, word));
                g_mic_active = true;
                active_start = last_voice = now_ms();
                dbg_counter = 0;
            }
        } else {
            // ===== MODO ATIVO: captando fala =====
            float   rms = chunk_rms(pcm, chunk);
            int64_t t   = now_ms();

            if (rms > VOICE_RMS_THRESHOLD) {
                last_voice = t;
            }

#if DEBUG_RMS
            // ~1 log por segundo (bloco de 512 amostras a 16 kHz = 32 ms)
            if (++dbg_counter >= 30) {
                dbg_counter = 0;
                ESP_LOGI(TAG, "rms=%.0f  silencio=%lld ms", rms, (long long)(t - last_voice));
            }
#endif

            // TODO: aqui você envia 'pcm' para o Core 1 (filtro 75-500 Hz + pYIN)
            //       e/ou grava o áudio original para a transcrição

            if ((t - last_voice) > SILENCE_TIMEOUT_MS ||
                (t - active_start) > MAX_ACTIVE_MS) {
                ESP_LOGI(TAG, "Silencio detectado -> mic desativado");
                g_mic_active = false;
            }
        }
    }
}

// Tarefa auxiliar no Core 1 (placeholder do pYIN / vibração)
void TarefaCore1(void *pvParameters)
{
    while (1) {
        ESP_LOGI(TAG_MAIN, "Core 1 | mic %s", g_mic_active ? "ATIVO" : "em espera");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "--- Inicializando Tarefas no ESP32-S3 ---");

    i2s_init();

    // WAKEWORD no Core 0
    xTaskCreatePinnedToCore(wakeword_task, "wakeword", 8 * 1024, NULL, 5, NULL, 0);

    // Tarefa auxiliar no Core 1
    xTaskCreatePinnedToCore(
        TarefaCore1,
        "Tarefa_Do_Core1",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}
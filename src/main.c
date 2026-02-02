#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_dsp.h"

const char *TAG = "ADC Trace: ";
const int cross_value = 2600;
const int validation_margin = 50;

#define LED_PIN1 7
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHAN ADC_CHANNEL_3

#define ATTENUATION ADC_ATTEN_DB_12
#define RESOLUTION ADC_BITWIDTH_12

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
int adc_read_voltage_rw(void);

#define SME_N 20
#define DELAY_MS 20
float  buffer[SME_N] __attribute__((aligned(16)));
float  weights[SME_N] __attribute__((aligned(16)));

int _index = 0;
inline int peek_index() {
    return (_index+1 >= SME_N) ? _index = 0 : _index + 1;    
}

inline int next_index() {

    if (++_index >= SME_N) {
        _index = 0;
    }
    return _index;
}

static inline void populate_buffer() { 
    buffer[next_index()] = (float)adc_read_voltage_rw(); 
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
}

void initialize_SMA_weights() {
    for (int i = 0; i < SME_N; i++) {
        weights[i] = 1.0f / SME_N;
    }
}

void setup(){
    gpio_config_t led_conf = {
        .pin_bit_mask = 1ULL << LED_PIN1,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN1, 1);

        // ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    // ADC channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = RESOLUTION,
        .atten = ATTENUATION,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHAN,
        .atten = ATTENUATION,
        .bitwidth = RESOLUTION,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle));

    ESP_LOGI(TAG, "ACD initialized");
}


int adc_read_voltage_rw(void) {
    int raw = 0;
    ESP_ERROR_CHECK(
        adc_oneshot_read(adc_handle, ADC_CHAN, &raw)
    );
    return raw;
}

typedef enum { UP_TREND, DOWN_TREND} trend_t;
// typedef enum { NO_CROSS, CROSS_UP, CROSS_DOWN } cross_t;

void app_main() {
    setup();
    initialize_SMA_weights();
    for (int i = 0; i < SME_N-1; i++) populate_buffer(); //pre-populate buffer
    float light_reading=0.0f;   

    trend_t old_trend, new_trend;
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Setup complete\n");

    gpio_set_level(LED_PIN1, 0);

    old_trend = buffer[peek_index()] < light_reading ? UP_TREND : DOWN_TREND;
    new_trend = old_trend;

    while (1) {        
        
        if (new_trend != old_trend) {
            if (new_trend == UP_TREND && (light_reading > cross_value + validation_margin)) {
                ESP_LOGI(TAG, "CROSS UP detected at %.2f", light_reading);
                gpio_set_level(LED_PIN1, 0);
            } else if (new_trend == DOWN_TREND && (light_reading < cross_value - validation_margin)) {
                ESP_LOGI(TAG, "CROSS DOWN detected at %.2f", light_reading);
                gpio_set_level(LED_PIN1, 1);
            }
            old_trend = new_trend;
        }

        populate_buffer();
        dsps_dotprod_f32(buffer, weights, &light_reading, (int)SME_N);
        ESP_LOGI(TAG, "ADC SMA Value: %.2f", light_reading);
        new_trend = buffer[peek_index()] < light_reading ? UP_TREND : DOWN_TREND;
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

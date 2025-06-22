#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include "motor_handler.h"

#define NUM_MOTORS 4
#define DSHOT_ESC_RESOLUTION_HZ 40000000

static const char *TAG = "Flight Controller";

static gpio_num_t motor_gpio[NUM_MOTORS] = {18, 19, 25, 26};
static rmt_channel_handle_t motor_chan[NUM_MOTORS] = {NULL};
static rmt_encoder_handle_t dshot_encoder = NULL;
static rmt_transmit_config_t tx_config = {
    .loop_count = -1, // infinite loop
};
static dshot_esc_throttle_t throttle[NUM_MOTORS];

#define DSHOT_ESC_RESOLUTION_HZ 40000000

void motor_init(void)
{
    ESP_LOGI(TAG, "Create RMT TX channels");
    for (int i = 0; i < NUM_MOTORS; i++) {
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = motor_gpio[i],
            .mem_block_symbols = 64,
            .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
            .trans_queue_depth = 10,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan[i]));
    }

    ESP_LOGI(TAG, "Install DSHOT ESC encoder");
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300
        .post_delay_us = 50,
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channels");
    for (int i = 0; i < NUM_MOTORS; i++) {
        ESP_ERROR_CHECK(rmt_enable(motor_chan[i]));
    }

    // Initialize throttle to 0
    for (int i = 0; i < NUM_MOTORS; i++) {
        throttle[i].throttle = 0;
        throttle[i].telemetry_req = false;
    }

    ESP_LOGI(TAG, "Start ESCs with zero throttle...");
    for (int i = 0; i < NUM_MOTORS; i++) {
        ESP_ERROR_CHECK(rmt_transmit(motor_chan[i], dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config));
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // 3s arming delay
}


// SET THROTTLE FUNCTION
void set_throttle(int motor_index, int val)
{
    if (motor_index < 0 || motor_index >= NUM_MOTORS) {
        ESP_LOGE(TAG, "Invalid motor index %d", motor_index);
        return;
    }

    if (val < 0) val = 0;
    if (val > 2047) val = 2047; // Max value for DSHOT

    throttle[motor_index].throttle = val;

    ESP_ERROR_CHECK(rmt_transmit(motor_chan[motor_index], dshot_encoder, &throttle[motor_index], sizeof(throttle[motor_index]), &tx_config));

    // Reset channel to keep ESC happy
    ESP_ERROR_CHECK(rmt_disable(motor_chan[motor_index]));
    ESP_ERROR_CHECK(rmt_enable(motor_chan[motor_index]));
}

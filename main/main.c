#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"

// Motor X pins
#define IN1 GPIO_NUM_7
#define IN2 GPIO_NUM_15
#define IN3 GPIO_NUM_18
#define IN4 GPIO_NUM_8

// Motor Y pins
#define IN1_Y GPIO_NUM_6
#define IN2_Y GPIO_NUM_4
#define IN3_Y GPIO_NUM_1
#define IN4_Y GPIO_NUM_2

// ADC Setup
#define ADC_CHANNEL_GPIO16   ADC_CHANNEL_5   // X axis pot
#define ADC_CHANNEL_GPIO17   ADC_CHANNEL_6   // Y axis pot
#define ADC_ATTEN            ADC_ATTEN_DB_12
#define BITWIDTH             ADC_BITWIDTH_12
#define DELAY_MS 20

// Limit Switches
#define LIMIT_SWITCH_1 GPIO_NUM_10  // X min
#define LIMIT_SWITCH_2 GPIO_NUM_11  // X max
#define LIMIT_SWITCH_3 GPIO_NUM_12  // (unused / spare)
#define LIMIT_SWITCH_4 GPIO_NUM_13  // Y min
#define LIMIT_SWITCH_5 GPIO_NUM_14  // Y max

// Buttons
#define CALLIBRATION GPIO_NUM_47
#define PEN_UP       GPIO_NUM_48

// LEDs
#define LED_YELLOW GPIO_NUM_37
#define LED_RED    GPIO_NUM_36
#define LED_GREEN  GPIO_NUM_35

// Calibration flag
int calibration_flag = 0;

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t cali_gpio16;
adc_cali_handle_t cali_gpio17;

// Stepper sequence (half-step)
const int step_matrix[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// X axis state
esp_timer_handle_t timer_x;
int step_index_x    = 0;
int current_position_x = 0;
int target_position_x  = 0;
int pot_min_mv_x    = 0;
int pot_max_mv_x    = 3300;
int max_steps_x     = 8096;

// Y axis state
esp_timer_handle_t timer_y;
int step_index_y    = 0;
int current_position_y = 0;
int target_position_y  = 0;
int pot_min_mv_y    = 0;
int pot_max_mv_y    = 3300;
int max_steps_y     = 8096;


void timer_callback_x(void* arg) {
    if (current_position_x == target_position_x) return;

    if (current_position_x < target_position_x) {
        step_index_x = (step_index_x + 1) % 8;
        current_position_x++;
    } else {
        step_index_x = (step_index_x - 1 + 8) % 8;
        current_position_x--;
    }

    gpio_set_level(IN1, step_matrix[step_index_x][0]);
    gpio_set_level(IN2, step_matrix[step_index_x][1]);
    gpio_set_level(IN3, step_matrix[step_index_x][2]);
    gpio_set_level(IN4, step_matrix[step_index_x][3]);
}

void timer_callback_y(void* arg) {
    if (current_position_y == target_position_y) return;

    if (current_position_y < target_position_y) {
        step_index_y = (step_index_y + 1) % 8;
        current_position_y++;
    } else {
        step_index_y = (step_index_y - 1 + 8) % 8;
        current_position_y--;
    }

    gpio_set_level(IN1_Y, step_matrix[step_index_y][0]);
    gpio_set_level(IN2_Y, step_matrix[step_index_y][1]);
    gpio_set_level(IN3_Y, step_matrix[step_index_y][2]);
    gpio_set_level(IN4_Y, step_matrix[step_index_y][3]);
}


void gpio_init() {
    // X motor
    gpio_reset_pin(IN1); gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN2); gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN3); gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN4); gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    // Y motor
    gpio_reset_pin(IN1_Y); gpio_set_direction(IN1_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN2_Y); gpio_set_direction(IN2_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN3_Y); gpio_set_direction(IN3_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN4_Y); gpio_set_direction(IN4_Y, GPIO_MODE_OUTPUT);

    // Limit switches
    gpio_reset_pin(LIMIT_SWITCH_1); gpio_set_direction(LIMIT_SWITCH_1, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_2); gpio_set_direction(LIMIT_SWITCH_2, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_3); gpio_set_direction(LIMIT_SWITCH_3, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_4); gpio_set_direction(LIMIT_SWITCH_4, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_5); gpio_set_direction(LIMIT_SWITCH_5, GPIO_MODE_INPUT);

    // Buttons
    gpio_reset_pin(CALLIBRATION); gpio_set_direction(CALLIBRATION, GPIO_MODE_INPUT);
    gpio_reset_pin(PEN_UP);       gpio_set_direction(PEN_UP,       GPIO_MODE_INPUT);

    // LEDs
    gpio_reset_pin(LED_YELLOW); gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_RED);    gpio_set_direction(LED_RED,    GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN);  gpio_set_direction(LED_GREEN,  GPIO_MODE_OUTPUT);
}


void calibration_scheme() {
    if (gpio_get_level(CALLIBRATION) == 0 && calibration_flag == 0) {
        calibration_flag = 1;
        gpio_set_level(LED_YELLOW, 1);

        int raw;

        // --- Calibrate X ---
        target_position_x = -99999;
        while (gpio_get_level(LIMIT_SWITCH_1) == 1) vTaskDelay(pdMS_TO_TICKS(10));
        target_position_x = current_position_x;
        current_position_x = 0;
        target_position_x  = 0;

        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw);
        adc_cali_raw_to_voltage(cali_gpio16, raw, &pot_min_mv_x);
        printf("Cal X: zero → pot = %d mV\n", pot_min_mv_x);

        target_position_x = 99999;
        while (gpio_get_level(LIMIT_SWITCH_2) == 1) vTaskDelay(pdMS_TO_TICKS(10));
        target_position_x = current_position_x;

        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw);
        adc_cali_raw_to_voltage(cali_gpio16, raw, &pot_max_mv_x);
        max_steps_x = current_position_x;
        printf("Cal X: max → pot = %d mV, steps = %d\n", pot_max_mv_x, max_steps_x);

        // --- Calibrate Y ---
        target_position_y = -99999;
        while (gpio_get_level(LIMIT_SWITCH_5) == 1) vTaskDelay(pdMS_TO_TICKS(10));
        target_position_y = current_position_y;
        current_position_y = 0;
        target_position_y  = 0;

        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO17, &raw);
        adc_cali_raw_to_voltage(cali_gpio17, raw, &pot_min_mv_y);
        printf("Cal Y: zero → pot = %d mV\n", pot_min_mv_y);

        target_position_y = 99999;
        while (gpio_get_level(LIMIT_SWITCH_4) == 1) vTaskDelay(pdMS_TO_TICKS(10));
        target_position_y = current_position_y;

        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO17, &raw);
        adc_cali_raw_to_voltage(cali_gpio17, raw, &pot_max_mv_y);
        max_steps_y = current_position_y;
        printf("Cal Y: max → pot = %d mV, steps = %d\n", pot_max_mv_y, max_steps_y);

        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_GREEN, 1);
    }
}


void app_main() {
    gpio_init();

    int raw16, mv16;
    int raw17, mv17;

    // ADC init
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_2 };
    adc_oneshot_new_unit(&init_config, &adc2_handle);

    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN, .bitwidth = BITWIDTH };
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_GPIO16, &config);
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_GPIO17, &config);

    // Calibration schemes for both channels
    adc_cali_curve_fitting_config_t cali16_cfg = {
        .unit_id = ADC_UNIT_2, .chan = ADC_CHANNEL_GPIO16,
        .atten = ADC_ATTEN,    .bitwidth = BITWIDTH
    };
    adc_cali_create_scheme_curve_fitting(&cali16_cfg, &cali_gpio16);

    adc_cali_curve_fitting_config_t cali17_cfg = {
        .unit_id = ADC_UNIT_2, .chan = ADC_CHANNEL_GPIO17,
        .atten = ADC_ATTEN,    .bitwidth = BITWIDTH
    };
    adc_cali_create_scheme_curve_fitting(&cali17_cfg, &cali_gpio17);

    // Timer X
    const esp_timer_create_args_t timer_args_x = {
        .callback = &timer_callback_x, .name = "stepper_x"
    };
    esp_timer_create(&timer_args_x, &timer_x);
    esp_timer_start_periodic(timer_x, 1000);

    // Timer Y
    const esp_timer_create_args_t timer_args_y = {
        .callback = &timer_callback_y, .name = "stepper_y"
    };
    esp_timer_create(&timer_args_y, &timer_y);
    esp_timer_start_periodic(timer_y, 1000);

    while (1) {
        calibration_scheme();

        // X axis
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw16);
        adc_cali_raw_to_voltage(cali_gpio16, raw16, &mv16);
        target_position_x = (mv16 - pot_min_mv_x) * max_steps_x / (pot_max_mv_x - pot_min_mv_x);
        if (target_position_x < 0)          target_position_x = 0;
        if (target_position_x > max_steps_x) target_position_x = max_steps_x;
        printf("X: %d mV -> target %d\n", mv16, target_position_x);

        // Y axis
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO17, &raw17);
        adc_cali_raw_to_voltage(cali_gpio17, raw17, &mv17);
        target_position_y = (mv17 - pot_min_mv_y) * max_steps_y / (pot_max_mv_y - pot_min_mv_y);
        if (target_position_y < 0)          target_position_y = 0;
        if (target_position_y > max_steps_y) target_position_y = max_steps_y;
        printf("Y: %d mV -> target %d\n", mv17, target_position_y);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
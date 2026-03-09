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

// ADC Setup
#define ADC_CHANNEL_GPIO16   ADC_CHANNEL_5  // X axis pot
#define ADC_ATTEN            ADC_ATTEN_DB_12
#define BITWIDTH             ADC_BITWIDTH_12
#define DELAY_MS 20

//Limit Switch Initialize
#define LIMIT_SWITCH_1 GPIO_NUM_10
#define LIMIT_SWITCH_2 GPIO_NUM_11
#define LIMIT_SWITCH_3 GPIO_NUM_12
#define LIMIT_SWITCH_4 GPIO_NUM_13

//Define switches
#define CALLIBRATION GPIO_NUM_47
#define PEN_UP GPIO_NUM_48

//Define LEDs
#define LED_YELLOW GPIO_NUM_37
#define LED_RED GPIO_NUM_36
#define LED_GREEN GPIO_NUM_35

//Define calibration flag
int calibration_flag = 0;

adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t cali_gpio16;

// Stepper motor sequence for forward and back
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

esp_timer_handle_t timer_x;   

// X axis
int step_index_x = 0;
int current_position_x = 0;
int target_position_x = 0;
int pot_min_mv = 0;
int pot_max_mv = 3300;
int max_steps_x = 8096; 
 

/// hardware callback, Gammie this is an interrupt bassically 

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


void gpio_init() {
    gpio_reset_pin(IN1); 
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN2); 
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN3); 
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN4); 
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LIMIT_SWITCH_1);
    gpio_set_direction(LIMIT_SWITCH_1, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_2);
    gpio_set_direction(LIMIT_SWITCH_2, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_3);
    gpio_set_direction(LIMIT_SWITCH_3, GPIO_MODE_INPUT);
    gpio_reset_pin(LIMIT_SWITCH_4);
    gpio_set_direction(LIMIT_SWITCH_4, GPIO_MODE_INPUT);
    gpio_reset_pin(CALLIBRATION);  
    gpio_set_direction(CALLIBRATION, GPIO_MODE_INPUT);
    gpio_reset_pin(PEN_UP);
    gpio_set_direction(PEN_UP, GPIO_MODE_INPUT);
    gpio_reset_pin(LED_YELLOW);
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_RED);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);       
}


void calibration_scheme(){
    if(gpio_get_level(CALLIBRATION) == 0 && calibration_flag == 0) { 
        calibration_flag = 1;
        gpio_set_level(LED_YELLOW, 1); // Turn on yellow LED to indicate calibration mode

        // Move X axis to limit switch 1 → this is physical zero
        target_position_x = -99999;  // tell the timer to drive backward indefinitely
        //**check*/
        while(gpio_get_level(LIMIT_SWITCH_1) == 1) {
            vTaskDelay(pdMS_TO_TICKS(10));  // wait, timer_callback_x is doing the stepping
        }
        target_position_x = current_position_x;  // stop the timer
        current_position_x = 0;                  // zero out position
        target_position_x = 0;

        // Record pot voltage at zero end
        int raw;
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw);
        adc_cali_raw_to_voltage(cali_gpio16, raw, &pot_min_mv);
        printf("Calibration: zero end → pot = %d mV\n", pot_min_mv);

        // Move X axis to limit switch 2 → this is physical max
        target_position_x = 99999;  // tell the timer to drive forward indefinitely
        while(gpio_get_level(LIMIT_SWITCH_2) == 1) {
            vTaskDelay(pdMS_TO_TICKS(10));  // wait, timer_callback_x is doing the stepping
        }
        target_position_x = current_position_x;  // stop the timer

        // Record pot voltage and real step count at max end
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw);
        adc_cali_raw_to_voltage(cali_gpio16, raw, &pot_max_mv);
        max_steps_x = current_position_x;  // real measured step count
        printf("Calibration: max end → pot = %d mV, steps = %d\n", pot_max_mv, max_steps_x);

        gpio_set_level(LED_YELLOW, 0);
        gpio_set_level(LED_GREEN, 1);
    }
}
void app_main() {

    gpio_init();
    int raw16;
    int mv16;


    // ADC initialization
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };
    adc_oneshot_new_unit(&init_config, &adc2_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_GPIO16, &config);

    adc_cali_curve_fitting_config_t cali16_cfg = {
        .unit_id = ADC_UNIT_2,
        .chan = ADC_CHANNEL_GPIO16,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_cali_create_scheme_curve_fitting(&cali16_cfg, &cali_gpio16);


    ///****SHIT I DON"T KNOW ****///

    // Timer init X
    const esp_timer_create_args_t timer_args_x = {
        .callback = &timer_callback_x,
        .name = "stepper_x"
    };
    esp_timer_create(&timer_args_x, &timer_x);
    esp_timer_start_periodic(timer_x, 1000);

    ///****SHIT I DON"T KNOW****///

    


    while(1) {
        calibration_scheme();
        // Read X pot
        adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw16);
        adc_cali_raw_to_voltage(cali_gpio16, raw16, &mv16);
        // Map pots to target positions
        target_position_x = (mv16 - pot_min_mv) * max_steps_x / (pot_max_mv - pot_min_mv);
        if (target_position_x < 0) target_position_x = 0;
        if (target_position_x > max_steps_x) target_position_x = max_steps_x;
        printf("X: %d mV -> target %d\n", mv16, target_position_x);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

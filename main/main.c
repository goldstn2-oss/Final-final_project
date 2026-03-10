#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO         (14)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits

//Set the PWM signal frequency required by servo motor
#define LEDC_FREQUENCY           50 // Frequency in Hertz. 

//Calculate the values for the minimum (0.75ms) and maximum (2.25) servo pulse widths
#define PEN_DOWN          615 // Set duty to 3.75%.
#define NEUTRAL              560 // Set duty to 7.5%.
#define ERASER_DOWN           475 // Set duty to 11.25%.

static void example_ledc_init(void);


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
#define LIMIT_SWITCH_3 GPIO_NUM_12  // Y min
#define LIMIT_SWITCH_4 GPIO_NUM_13  // Y max

// Switches
#define CALLIBRATION GPIO_NUM_47
#define PEN_ERASE_DOWN      GPIO_NUM_20 
#define PEN_ERASE_NEUTRAL   GPIO_NUM_21

// LEDs
#define LED_YELLOW GPIO_NUM_37
#define LED_GREEN  GPIO_NUM_36
#define LED_RED    GPIO_NUM_35

// Calibration values
int x_min= 0; 
int x_max = 0;
int y_min = 0;
int y_max = 0;
int callibration = 0;

//ADC config
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t cali_gpio16;
adc_cali_handle_t cali_gpio17;

// Stepper motor sequence
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
int step_index_x       = 0;
int current_position_x = 0;
int target_position_x  = 0;
int pot_max_mv_x   = 5003; // Max mV from pot at max position, used for mapping pot reading to position

// Y axis state
esp_timer_handle_t timer_y;
int step_index_y       = 0;
int current_position_y = 0;
int target_position_y  = 0;
int pot_max_mv_y   = 5003; // Max mV from pot at max position, used for mapping pot reading to position


void timer_callback_x(void* arg) {

    if (current_position_x == target_position_x) 
        return;
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

    if (current_position_y == target_position_y) 
        return;
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

void gpio_init(void);
void calibration(void);
void servo(void);

void app_main() {

    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 3.75% (0 degrees)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, NEUTRAL);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    gpio_init();

    int raw16, mv16;
    int raw17, mv17;

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_2 };
    adc_oneshot_new_unit(&init_config, &adc2_handle);

    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN, .bitwidth = BITWIDTH };
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_GPIO16, &config);
    adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_GPIO17, &config);

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

    ///Create timer for X axis control
    const esp_timer_create_args_t timer_args_x = { .callback = &timer_callback_x, .name = "stepper_x" };
    esp_timer_create(&timer_args_x, &timer_x);
    esp_timer_start_periodic(timer_x, 1000);

    const esp_timer_create_args_t timer_args_y = { .callback = &timer_callback_y, .name = "stepper_y" };
    esp_timer_create(&timer_args_y, &timer_y);
    esp_timer_start_periodic(timer_y, 1000);


    while (1) {

        if (gpio_get_level(CALLIBRATION) == 0 && callibration == 0) {
            gpio_set_level(LED_YELLOW, 1);
            calibration();
            gpio_set_level(LED_YELLOW, 0);
            printf("Calibration complete\n");
            gpio_set_level(LED_GREEN, 1);
            callibration = 1;
            printf("Calibration complete\n");
        }

        if (gpio_get_level(CALLIBRATION) == 0 && callibration == 1) {

            adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO16, &raw16);
            adc_cali_raw_to_voltage(cali_gpio16, raw16, &mv16);

            target_position_x = (mv16) * x_max / (pot_max_mv_x);

            if (target_position_x < 0){
                target_position_x = 0;
            }
            if (target_position_x > x_max){
                target_position_x = x_max - 1;
            }

            printf("X: %d mV -> target %d\n", mv16, target_position_x);

            adc_oneshot_read(adc2_handle, ADC_CHANNEL_GPIO17, &raw17);
            adc_cali_raw_to_voltage(cali_gpio17, raw17, &mv17);

            target_position_y = (mv17) * y_max / (pot_max_mv_y);

            if (target_position_y < 0){
                target_position_y = 0;
            }
            if (target_position_y > y_max){
                target_position_y = y_max - 1;
            }
            printf("Y: %d mV -> target %d\n", mv17, target_position_y);
            servo();
        }

        if (gpio_get_level(CALLIBRATION) == 1 && callibration == 1) {
                callibration = 0;
                gpio_set_level(LED_GREEN, 0);
                gpio_set_level(LED_YELLOW, 0);
                gpio_set_level(LED_RED, 0);
                target_position_x = 0;
                target_position_y = 0;
            printf("System OFF\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}


void servo(void){
    
    if (gpio_get_level(PEN_ERASE_NEUTRAL) == 0) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, NEUTRAL);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        gpio_set_level(LED_RED, 1);
        printf("Neutral\n");
    }
    else {
        gpio_set_level(LED_RED, 0);
        int eraser_down = gpio_get_level(PEN_ERASE_DOWN);
        printf("%d\n", eraser_down);
        if (gpio_get_level(PEN_ERASE_DOWN) == 0) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, ERASER_DOWN);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            printf("Eraser down\n");
        }
        else {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, PEN_DOWN);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            printf("Pen down\n");
        }
    }
 
    
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
ledc_channel_config(&ledc_channel);
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

    gpio_reset_pin(IN1_Y); 
    gpio_set_direction(IN1_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN2_Y); 
    gpio_set_direction(IN2_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN3_Y); 
    gpio_set_direction(IN3_Y, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN4_Y); 
    gpio_set_direction(IN4_Y, GPIO_MODE_OUTPUT);

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
    gpio_reset_pin(PEN_ERASE_DOWN);       
    gpio_set_direction(PEN_ERASE_DOWN, GPIO_MODE_INPUT);
    gpio_reset_pin(PEN_ERASE_NEUTRAL);    
    gpio_set_direction(PEN_ERASE_NEUTRAL, GPIO_MODE_INPUT);

    gpio_reset_pin(LED_YELLOW); 
    gpio_set_direction(LED_YELLOW, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_RED);    
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN);    
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
}


void calibration(void) {
    //set pen up
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, NEUTRAL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    printf("Neutral\n");
    
    //x_min
    printf("Calibrating X axis...\n");
    target_position_x = -99999;
    while (gpio_get_level(LIMIT_SWITCH_1) == 1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    current_position_x = 0;
    target_position_x = 0;
    x_min = current_position_x;
    printf("X min: %d\n", x_min);

    target_position_x = 99999;
    while (gpio_get_level(LIMIT_SWITCH_2) == 1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    x_max = current_position_x;
    target_position_x = x_max - 1;
    printf("X max: %d\n", x_max);

     //y_min
    printf("Calibrating Y axis...\n");
    target_position_y = -99999;
    while (gpio_get_level(LIMIT_SWITCH_4) == 1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    current_position_y = 0;
    target_position_y = 0;
    y_min = current_position_y;
    printf("Y min: %d\n", y_min);

    target_position_y = 99999;
    while (gpio_get_level(LIMIT_SWITCH_3) == 1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    y_max = current_position_y;
    target_position_y = y_max - 1;
    printf("Y max: %d\n", y_max);

        return; 
    }
    

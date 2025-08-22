#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

// Motor pins (use your wiring)
#define LEFT_LPWM   GPIO_NUM_6   // PWM pin for left forward
#define LEFT_RPWM   GPIO_NUM_7   // PWM pin for left reverse
#define RIGHT_LPWM  GPIO_NUM_5   // PWM pin for right forward
#define RIGHT_RPWM  GPIO_NUM_4   // PWM pin for right reverse

// Optional: EN pins (if you wired R_EN/L_EN to ESP pins instead of tying to VCC)
#define LEFT_LEN    GPIO_NUM_15  // example - change or remove if you tied EN to VCC
#define LEFT_REN    GPIO_NUM_16
#define RIGHT_LEN   GPIO_NUM_17
#define RIGHT_REN   GPIO_NUM_18

// PWM settings
#define PWM_FREQ_HZ       1000
#define PWM_RESOLUTION    LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY      ((1 << 8) - 1)   // 255
#define PWM_FULL_DUTY     PWM_MAX_DUTY     // run at full power

void init_pwm_channels(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch = {0};
    // map channel 0 -> LEFT_LPWM
    ch.channel = LEDC_CHANNEL_0;
    ch.gpio_num = LEFT_LPWM;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;
    ch.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 1 -> LEFT_RPWM
    ch.channel = LEDC_CHANNEL_1;
    ch.gpio_num = LEFT_RPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 2 -> RIGHT_LPWM
    ch.channel = LEDC_CHANNEL_2;
    ch.gpio_num = RIGHT_LPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // channel 3 -> RIGHT_RPWM
    ch.channel = LEDC_CHANNEL_3;
    ch.gpio_num = RIGHT_RPWM;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    // ensure all duties start at 0
    for (int c = 0; c < 4; ++c) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c);
    }
}

void enable_bts_en_pins(void) {
    // If you tied EN to VCC physically, you can skip this function.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<LEFT_LEN) | (1ULL<<LEFT_REN) | (1ULL<<RIGHT_LEN) | (1ULL<<RIGHT_REN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Drive EN pins HIGH (enable drivers)
    gpio_set_level(LEFT_LEN, 1);
    gpio_set_level(LEFT_REN, 1);
    gpio_set_level(RIGHT_LEN, 1);
    gpio_set_level(RIGHT_REN, 1);
}

void run_motors_full_forward(void) {
    // LEFT side: LPWM = PWM, RPWM = 0  -> left forward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_FULL_DUTY); // LEFT_LPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);            // LEFT_RPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // RIGHT side: invert signals so right wheels also move forward
    // RIGHT_LPWM = 0, RIGHT_RPWM = PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);            // RIGHT_LPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, PWM_FULL_DUTY); // RIGHT_RPWM
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void app_main(void) {
    // init PWM channels
    init_pwm_channels();

    // init EN pins (optional - skip if EN tied to VCC)
    enable_bts_en_pins();

    // Give a moment for everything to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    // Run motors full speed forward continuously
    run_motors_full_forward();

    // Do nothing — motors keep running until you cut power. If you want to keep the MCU alive:
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // idle
    }
}
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "DRV8871.h"

#include "driver/gpio.h"

DRV8871::DRV8871(gpio_num_t pin1, gpio_num_t pin2) {
    this->pin1 = pin1;
    this->pin2 = pin2;

    ESP_LOGI(DRV8871::TAG, "Initializing motor");
    this->duty_cycle = 0.0;

    this->init_timer();
    this->init_operator();
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(this->motor_operator, this->motor_timer));

    this->init_comparator();
    this->init_generators();

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
                      this->motor_generator_a,
                      MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
                      this->motor_generator_b,
                      MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));

    this->brake();

    mcpwm_timer_enable(motor_timer);
    mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP);
}

/**
 * @brief Initialize the comparator used to set the duty cycle
 * 
 */
void DRV8871::init_comparator(void) {
    mcpwm_comparator_config_t motor_comparator_config;
    motor_comparator_config.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_comparator(this->motor_operator, &motor_comparator_config, &(this->motor_comparator)));
    mcpwm_comparator_set_compare_value(this->motor_comparator, 0);
}

/**
 * @brief Initialize the main timer
 * 
 */
void DRV8871::init_timer(void) {
    mcpwm_timer_config_t motor_timer_config;
    motor_timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    motor_timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    motor_timer_config.group_id = 0;
    motor_timer_config.intr_priority = 0;
    motor_timer_config.resolution_hz = 10000000; // 10MHz, 1 tick = 0.1us
    motor_timer_config.period_ticks = 800; // 50us, 20KHz
    ESP_ERROR_CHECK(mcpwm_new_timer(&motor_timer_config, &(this->motor_timer)));
}

/**
 * @brief Initialize the main operator
 * 
 */
void DRV8871::init_operator(void) {
    mcpwm_operator_config_t motor_operator_config;
    motor_operator_config.group_id = 0;
    motor_operator_config.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&motor_operator_config, &(this->motor_operator)));
}

/**
 * @brief Initialize the main operator
 * 
 */
void DRV8871::init_generators(void) {
    mcpwm_generator_config_t motor_generator_config_a;
    motor_generator_config_a.gen_gpio_num = this->pin1;
    motor_generator_config_a.flags.invert_pwm = false;
    motor_generator_config_a.flags.io_loop_back = false;
    motor_generator_config_a.flags.io_od_mode = false;
    motor_generator_config_a.flags.pull_down = false;
    motor_generator_config_a.flags.pull_up = false;
    mcpwm_new_generator(this->motor_operator, &motor_generator_config_a, &(this->motor_generator_a));

    mcpwm_generator_config_t motor_generator_config_b;
    motor_generator_config_b.gen_gpio_num = this->pin2;
    motor_generator_config_b.flags.invert_pwm = false;
    motor_generator_config_b.flags.io_loop_back = false;
    motor_generator_config_b.flags.io_od_mode = false;
    motor_generator_config_b.flags.pull_down = false;
    motor_generator_config_b.flags.pull_up = false;
    mcpwm_new_generator(this->motor_operator, &motor_generator_config_b, &(this->motor_generator_b));
}

/**
 * @brief Stop the motor by holding in the brake state
 * 
 */
void DRV8871::brake(void) {
    ESP_LOGI(DRV8871::TAG, "Setting brake mode");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_a,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_a,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_KEEP))); 

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_b,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_b,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_KEEP))); 
}

/**
 * @brief Stop the motor by holding in the brake state
 * 
 */
void DRV8871::coast(void) {
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_a,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_a,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_KEEP))); 

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_b,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_b,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_KEEP))); 
}

/**
 * @brief Set the reverse direction by oscillating pin 1 and holding pin 2 high.
 * 
 */
void DRV8871::reverse(void) {
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_a,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_a,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW))); 

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_b,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_b,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_KEEP))); 
}

/**
 * @brief Set the forward direction by holding pin 1 high and oscillating pin 2
 * 
 */
void DRV8871::forward(void) {
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_a,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_a,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_HIGH))); 

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator_b,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator_b,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW))); 
}

/**
 * @brief Sets the duty cycle for the motor pwm signal. Range allowed is 0-1. May change this later to set speed and direction 
 * in a single command.
 * 
 * @param duty_cycle Duty cycle level to set, as a fraction between 0 and 1.
 */
void DRV8871::set_duty_cycle(double duty_cycle) {
    if(duty_cycle < 0) { duty_cycle = 0; }
    if(duty_cycle > 1) { duty_cycle = 1; }
    this->duty_cycle = duty_cycle;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparator, (unsigned int)((1.0-this->duty_cycle)*(800.0))));
}

/**
 * @brief Set the speed (and direction) of the motor
 * 
 * @param speed Signed int8 where:
 *      0 indicates stop
 *      positive values indicate forward, and duty cycle is speed/INT8_MAX
 *      negative values indicate reverse, and duty cycle is speed/INT8_MIN
 */
void DRV8871::setSpeed(int8_t speed) {
    if(speed == 0) { 
        this->brake();
    } else if(speed > 0) {
        this->forward();
        this->set_duty_cycle((double)(speed / (double)INT8_MAX));
    } else {
        this->reverse();
        this->set_duty_cycle((double)(speed / (double)INT8_MIN));
    }
}

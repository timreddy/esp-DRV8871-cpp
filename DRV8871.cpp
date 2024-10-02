#include <stdio.h>
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "DRV8871.h"

DRV8871::DRV8871(void) {
    this->duty_cycle = 0.0;
    this->on = false;
    this->direction = 1;   // 0 = reverse, 1 = forward
    this->brake_mode = 0;  // 0 = coast, 1 = brake

    this->init_timer();
    this->init_operator();
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(this->motor_operator, this->motor_timer));

    this->init_comparator();

    // This all should be moved to functions to set the direction and duty cycle.
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                      this->motor_generator,
                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                      this->motor_generator,
                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
                      this->motor_generator,
                      MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));

    mcpwm_timer_enable(motor_timer);


    mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP);
}

DRV8871::DRV8871(gpio_num_t pin1, gpio_num_t pin2) : DRV8871() {
    this->set_pins(pin1, pin2);
}

void DRV8871::init_comparator(void) {
    mcpwm_comparator_config_t motor_comparator_config;
    motor_comparator_config.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_comparator(this->motor_operator, &motor_comparator_config, &(this->motor_comparator)));
    mcpwm_comparator_set_compare_value(this->motor_comparator, 0);
}

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

void DRV8871::init_operator(void) {
    mcpwm_operator_config_t motor_operator_config;
    motor_operator_config.group_id = 0;
    motor_operator_config.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&motor_operator_config, &(this->motor_operator)));
}

/**
 * @brief Set the gpio pins. This needs to be done before initializing the mcpwm controller.
 * 
 * @param pin1 
 * @param pin2 
 */
void DRV8871::set_pins(gpio_num_t pin1, gpio_num_t pin2) {
    this->pin1 = pin1;
    this->pin2 = pin2;

    ESP_ERROR_CHECK(gpio_reset_pin(this->pin1));
    ESP_ERROR_CHECK(gpio_set_direction(this->pin1,GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(this->pin1, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(this->pin1,0));

    ESP_ERROR_CHECK(gpio_reset_pin(this->pin2));
    ESP_ERROR_CHECK(gpio_set_direction(this->pin2,GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(this->pin2, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(this->pin2,0));
}

void DRV8871::start(void) {
    //the following line should be replaces with a proper mcpwm output
    ESP_ERROR_CHECK(gpio_set_level(this->pin1,1));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(this->motor_timer, MCPWM_TIMER_START_NO_STOP));
    this->on = true;
}

void DRV8871::stop(void) {
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(this->motor_timer, MCPWM_TIMER_STOP_FULL));

    //the following two lines should be replaces with a proper mcpwm output
    ESP_ERROR_CHECK(gpio_set_level(this->pin1,0));
    ESP_ERROR_CHECK(gpio_set_level(this->pin2,0));

    this->on = false;
}

void DRV8871::forward(void) {
    this->direction = 1;
}

void DRV8871::reverse(void) {
    this->direction = 0;
}

/**
 * @brief Sets the duty cycle for the motor pwm signal. Range allowed is 0-100. Values over 100 are capped at 100
 * 
 * @param duty_cycle Duty cycle level to set, as a percent. E.g. 0 = 0% (stopped) and 100 = 100% (full speed). Values over 100 are capped. 
 */
void DRV8871::set_duty_cycle(double duty_cycle) {
    if(duty_cycle < 0) { duty_cycle = 0; }
    if(duty_cycle > 1) { duty_cycle = 1; }
    this->duty_cycle = duty_cycle;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparator, (unsigned int)((1.0-this->duty_cycle)*(800.0))));
}

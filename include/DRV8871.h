#pragma once

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

class DRV8871 {
    private:
        gpio_num_t  pin1, pin2;

        double duty_cycle;  // range [0.0 - 1.0]
        bool   on;
        bool   direction;   // 0 = reverse, 1 = forward
        bool   brake_mode;  // 0 = coast, 1 = brake

        //mcpwm components
        mcpwm_timer_handle_t    motor_timer;
        mcpwm_oper_handle_t     motor_operator;
        mcpwm_cmpr_handle_t     motor_comparator;
        mcpwm_gen_handle_t      motor_generator;


        void set_pins(gpio_num_t pin1, gpio_num_t pin2);
        void init_timer(void);
        void init_operator(void);
        void init_comparator(void);

    public:

        DRV8871(void);
        DRV8871(gpio_num_t pin1, gpio_num_t pin2);


        void start(void);
        void stop(void);

        void forward(void);
        void reverse(void);
        void set_duty_cycle(double duty_cycle);
};

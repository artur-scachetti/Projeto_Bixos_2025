#ifndef PID_H
#define PID_H

#include "h_bridge.h"
#include "encoder.h"

#define MAX_PWM                 1023
#define FREQ_COMUNICATION       50

typedef struct {

    float kp;
    float ki;
    float kd;

    float conversion_rate;
    float target_rpm;

    float setpoint;
    float integral;
    float previous_error;

    float output_max;

}pid_ctrl_block_t;

typedef pid_ctrl_block_t* pid_ctrl_block_handle_t;

pid_ctrl_block_handle_t init_pid(motor_side_t motor);

void pid_calculate(pcnt_unit_handle_t left_encoder, pid_ctrl_block_handle_t left_pid, pcnt_unit_handle_t right_encoder, 
                   pid_ctrl_block_handle_t right_pid);

#endif
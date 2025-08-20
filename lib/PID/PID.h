#ifndef PID_H
#define PID_H

#include "h_bridge.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_check.h"

#define KP_R 0
#define KI_R 0
#define KD_R 0
#define MAX_OUTPUT_R 0
#define MIN_OUTPUT_R 0 
#define MAX_INTEGRAL_R 0
#define MIN_INTEGRAL_R 0

#define KP_L 0
#define KI_L 0
#define KD_L 0
#define MAX_OUTPUT_L 0
#define MIN_OUTPUT_L 0
#define MAX_INTEGRAL_L 0 
#define MIN_INTEGRAL_L 0

#define kp(MOTOR) (MOTOR == RIGHT_MOTOR)? KP_R : KP_L
#define ki(MOTOR) (MOTOR == RIGHT_MOTOR)? KI_R : KI_L
#define kd(MOTOR) (MOTOR == RIGHT_MOTOR)? KD_R : KD_L
#define max_output(MOTOR) (MOTOR == RIGHT_MOTOR)? MAX_OUTPUT_R : MAX_OUTPUT_L
#define min_output(MOTOR) (MOTOR == RIGHT_MOTOR)? MIN_OUTPUT_R : MIN_OUTPUT_L
#define max_integral(MOTOR) (MOTOR == RIGHT_MOTOR)? MAX_INTEGRAL_R : MAX_INTEGRAL_L
#define min_integral(MOTOR) (MOTOR == RIGHT_MOTOR)? MIN_INTEGRAL_R : MIN_INTEGRAL_L

#define MAX_PWM                 0
#define FREQ_COMUNICATION       0

typedef enum {
    PID_CAL_TYPE_INC,
    PID_CAL_TYPE_POS,  
} pid_calculate_type_t;

typedef float (*pid_calc_func_t)(pid_ctrl_block_t *pid, float error);

typedef struct {

    float kp;
    float ki;
    float kd;

    float previous_error1;
    float previous_error2;
    float integral_error;

    float last_output;
    float min_output;
    float max_output;

    float min_integral;
    float max_integral;

    float conversion_rate;
    float target_rpm;

    float setpoint;

    pid_calc_func_t calculate_func;

}pid_ctrl_block_t;

typedef struct {
    float kp;                      // PID Kp parameter
    float ki;                      // PID Ki parameter
    float kd;                      // PID Kd parameter
    float max_output;              // PID maximum output limitation
    float min_output;              // PID minimum output limitation
    float max_integral;            // PID maximum integral value limitation
    float min_integral;            // PID minimum integral value limitation
    pid_calculate_type_t cal_type; // PID calculation type

} pid_ctrl_parameter_t;

typedef struct {
    pid_ctrl_parameter_t init_param; 

} pid_ctrl_config_t;


typedef pid_ctrl_block_t* pid_ctrl_block_handle_t;

esp_err_t pid_new_control_block(const pid_ctrl_config_t *config, pid_ctrl_block_handle_t *status_pid);

esp_err_t pid_update_parameters(pid_ctrl_block_handle_t pid, const pid_ctrl_parameter_t *params);

esp_err_t pid_compute(pid_ctrl_block_handle_t pid, float input_error, float *ret_result);

esp_err_t pid_reset_ctrl_block(pid_ctrl_block_handle_t pid);

esp_err_t pid_delete_ctrl_block(pid_ctrl_block_handle_t pid);

pid_ctrl_block_handle_t init_pid(motor_side_t motor);

static float pid_calculate_positional(pid_ctrl_block_t *pid, float error);

static float pid_calculate_incremental(pid_ctrl_block_t *pid, float error);

esp_err_t pid_calculate(pid_ctrl_block_handle_t pid, motor_side_t motor, float target_rpm, float* inc_value);

#endif
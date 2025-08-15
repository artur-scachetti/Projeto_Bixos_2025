#include "PID.h"

const char *TAG_PID_L = "LEFT PID";
const char *TAG_PID_R = "RIGHT PID";

//Inicializa e configura o PID (retorna o 'pid_ctrl_block_handle_t' pronto)
pid_ctrl_block_handle_t init_pid(motor_side_t motor)
{
    pid_ctrl_block_handle_t pid = malloc(sizeof(pid_ctrl_block_t));

    pid->kp = 1.0;
    pid->ki = 0.1;
    pid->kd = 0.05;

    pid->conversion_rate = 10;
    pid->target_rpm = 180;

    pid->setpoint = 0;
    pid->integral = 0;
    pid->previous_error = 0;

    pid->output_max = 1023;

    return pid;
}

//Aplica o controle PID sobre o erro do RPM, atualizando o PMW dos motores
void pid_calculate(pcnt_unit_handle_t left_encoder, pid_ctrl_block_handle_t left_pid, pcnt_unit_handle_t right_encoder, 
                   pid_ctrl_block_handle_t right_pid)
{
    double interval = 2 * FREQ_COMUNICATION / 1000.0; //Converte a frequência de comunicação (100ms) p/ segundos
    
    float left_pulses = pulse_count(left_encoder);
    float right_pulses = pulse_count(right_encoder);

    float left_rpm = left_pulses*left_pid->conversion_rate;
    float right_rpm = right_pulses*right_pid->conversion_rate;

    float left_error = left_pid->target_rpm - left_rpm;
    float right_error = right_pid->target_rpm - right_rpm;

    left_pid->integral += left_error*interval;
    if (left_pid->integral > MAX_PWM)
        left_pid->integral = MAX_PWM;

    else if (left_pid->integral < -MAX_PWM)
        left_pid->integral = -MAX_PWM;

    right_pid->integral += right_error*interval;
    if (right_pid->integral > MAX_PWM)
        right_pid->integral = MAX_PWM;

    else if (right_pid->integral < -MAX_PWM)
        right_pid->integral = -MAX_PWM;

    float left_derivative = (left_error - left_pid->previous_error)/interval;
    float right_derivative = (right_error - right_pid->previous_error)/interval;

    float left_pwm_output = (int)(left_pid->kp*left_error + left_pid->ki*left_pid->integral + left_pid->kd*left_derivative);
        if (left_pwm_output > MAX_PWM)
            left_pwm_output = MAX_PWM;

        else if (left_pwm_output < -MAX_PWM)
            left_pwm_output = -MAX_PWM;
    
    float right_pwm_output = (int)(right_pid->kp*right_error + right_pid->ki*right_pid->integral + right_pid->kd*right_derivative);
        if (right_pwm_output > MAX_PWM)
            right_pwm_output = MAX_PWM;

        else if (right_pwm_output < -MAX_PWM)
            right_pwm_output = -MAX_PWM;

    ESP_LOGI(TAG_PID_L, "Target RPM: %.2f | RPM: %.2f | PWM: %.2f", left_pid->target_rpm, left_rpm, left_pwm_output);
    ESP_LOGI(TAG_PID_R, "Target RPM: %.2f | RPM: %.2f | PWM: %.2f", right_pid->target_rpm, right_rpm, right_pwm_output);

    update_motor(LEFT_MOTOR, left_pwm_output);
    update_motor(RIGHT_MOTOR, right_pwm_output);
}

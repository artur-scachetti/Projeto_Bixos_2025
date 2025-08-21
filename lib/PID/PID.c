#include "PID.h"

const char *TAG_PID = "PID";
const char *TAG_PID_CTRL = "PID CTRL";

esp_err_t pid_new_control_block(const pid_ctrl_config_t *config, pid_ctrl_block_handle_t *status_pid)
{
    esp_err_t ret = ESP_OK;
    pid_ctrl_block_t *pid = NULL;

    /* Check the input pointer */
    ESP_GOTO_ON_FALSE(config && status_pid, ESP_ERR_INVALID_ARG, err, TAG_PID_CTRL, "invalid argument");

    pid = calloc(1, sizeof(pid_ctrl_block_t));
    ESP_GOTO_ON_FALSE(pid, ESP_ERR_NO_MEM, err, TAG_PID_CTRL, "not enough memory");
    ESP_GOTO_ON_ERROR(pid_update_parameters(pid, &config->init_param), err, TAG_PID_CTRL, "init PID parameters failed");

    *status_pid = pid;
    return ret;

    err:
    if (pid) {
        free(pid);
    }
    return ret;
}

esp_err_t pid_update_parameters(pid_ctrl_block_handle_t pid, const pid_ctrl_parameter_t *params)
{
    ESP_RETURN_ON_FALSE(pid && params, ESP_ERR_INVALID_ARG, TAG_PID_CTRL, "invalid argument");
    pid->kp = params->kp;
    pid->ki = params->ki;
    pid->kd = params->kd;
    pid->max_output = params->max_output;
    pid->min_output = params->min_output;
    pid->max_integral = params->max_integral;
    pid->min_integral = params->min_integral;
    /* Set the calculate function according to the PID type */
    switch (params->cal_type) {
    case PID_CAL_TYPE_INC:
        pid->calculate_func = pid_calc_incremental();
        break;
    case PID_CAL_TYPE_POS:
        pid->calculate_func = pid_calc_positional();
        break;
    default:
        ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_ARG, TAG_PID_CTRL, "invalid PID calculation type:%d", params->cal_type);
    }
    return ESP_OK;
}

esp_err_t pid_compute(pid_ctrl_block_handle_t pid, float input_error, float *ret_result)
{
    ESP_RETURN_ON_FALSE(pid && ret_result, ESP_ERR_INVALID_ARG, TAG_PID_CTRL, "invalid args");

    *ret_result = pid->calculate_func(pid, input_error);

    return ESP_OK;
}

esp_err_t pid_reset_ctrl_block(pid_ctrl_block_handle_t pid)
{
    ESP_RETURN_ON_FALSE(pid, ESP_ERR_INVALID_ARG, TAG_PID_CTRL, "invalid arg");

    pid->integral_error = 0;
    pid->last_output = 0;
    pid->previous_error1 = 0;
    pid->previous_error2 = 0;

    return ESP_OK;
}

esp_err_t pid_delete_ctrl_block(pid_ctrl_block_handle_t pid)
{
    ESP_RETURN_ON_FALSE(pid, ESP_ERR_INVALID_ARG, TAG_PID_CTRL, "invalid arg");

    free(pid);

    return ESP_OK;
}

//Inicializa e configura o PID (retorna o 'pid_ctrl_block_handle_t' pronto)
pid_ctrl_block_handle_t init_pid(motor_side_t motor)
{

    pid_ctrl_parameter_t pid_param = {

        .kp = kp(motor),
        .ki = ki(motor),
        .kd = kd(motor),

        .max_integral = max_integral(motor),
        .min_integral = min_integral(motor),

        .max_output = max_output(motor),
        .min_output = min_output(motor),

        .cal_type = PID_CAL_TYPE_INC
    };

    pid_ctrl_config_t pid_ctrl_config = {

        .init_param = pid_param
    };

    pid_ctrl_block_handle_t pid;

    ESP_ERROR_CHECK(pid_new_control_block(&pid_param, &pid));
    return pid;
}

static float pid_calculate_positional(pid_ctrl_block_t *pid, float error)
{
    float output = 0;

    pid->integral_error += error;

    pid->integral_error = MIN(pid->integral_error, pid->max_integral);
    pid->integral_error = MAX(pid->integral_error, pid->min_integral);

    output = error*pid->kp + (error - pid->previous_error1)*pid->kd + pid->integral_error*pid->ki;

    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    pid->previous_error1 = error;

    return output;
}

static float pid_calculate_incremental(pid_ctrl_block_t *pid, float error)
{
    float output = 0;

    output = (error - pid->previous_error1) * pid->kd + (error - 2*pid->previous_error1 + pid->previous_error2) * pid->kd 
             + error *pid->ki + pid->last_output;

    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    pid->previous_error2 = pid->previous_error1;
    pid->previous_error1 = error;
    
    pid->last_output = output;

    return output;
}

void PWM_limit(float* PWM)
{
    if(*PWM > 1023)
    {
        *PWM = 1023;
    }
    else if(*PWM < -1023)
    {
        *PWM = -1023;
    }
}
//Aplica o controle PID sobre o erro do RPM, atualizando o PMW dos motores
esp_err_t pid_calculate(pid_ctrl_block_handle_t pid, motor_side_t motor, float target_rpm, float* inc_value)
{
    encoder_side_t encoder = ((motor == RIGHT_MOTOR) ? ENCODER_RIGHT : ENCODER_LEFT);

    float conversion_rate = 0;
    float current_rpm = pulse_count(encoder) * conversion_rate;

    float error = target_rpm - current_rpm;

    float value;

    ESP_ERROR_CHECK(pid_compute(pid, error, &value));
    value *= (1/conversion_rate);
    *inc_value += value;

    PWM_limit(inc_value);

    update_motor(motor, *inc_value);

    ESP_LOGI(TAG_PID, "Alvo: %f, | Erro: %f, | PID: %f\n", target_rpm, error, *inc_value);

    return ESP_OK;
}

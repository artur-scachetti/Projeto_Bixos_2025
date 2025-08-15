#include "PID.h"
#include "I2C.h"

//Inicializa as filas para troca de dados entre o I2C e o PID
QueueHandle_t rpm_queue;
QueueHandle_t target_rpm_queue;

pcnt_unit_handle_t left_encoder;
pcnt_unit_handle_t right_encoder;

void init_all()
{
    init_gpio(LEFT_MOTOR);
    init_gpio(RIGHT_MOTOR);

    init_pwm(LEFT_MOTOR);
    init_pwm(RIGHT_MOTOR);

    left_encoder = init_encoder(ENCODER_LEFT);
    right_encoder = init_encoder(ENCODER_RIGHT);

}

void task_motor_control()
{
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);

    target_rpm_data_t target_rpm;

    while(1)
    {
        if(xQueueReceive(target_rpm_queue, &target_rpm, 0) == pdPASS)
        {
            left_pid->target_rpm = target_rpm.target_left_rpm;
            right_pid->target_rpm = target_rpm.target_right_rpm;
        }

        pid_calculate(left_encoder, left_pid, right_encoder, right_pid);

        vTaskDelay(pdMS_TO_TICKS(2 * FREQ_COMUNICATION));
    }
}

void i2c_task_com()
{
    rpm_data_t last_rpm = {0,0};
    target_rpm_data_t last_target_rpm = {0,0};
    
    ESP_ERROR_CHECK(init_i2c());

    while(1)
    {
        i2c_read_task(last_target_rpm);
        i2c_write_task(last_rpm);
        
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMUNICATION));
    }
}

void app_main()
{
    init_all();

    rpm_queue = xQueueCreate(10, sizeof(rpm_data_t));
    target_rpm_queue = xQueueCreate(10, sizeof(target_rpm_data_t));
    
    xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(i2c_task_com, "i2c_task_com", 4096, NULL, 1, NULL, 1);

}






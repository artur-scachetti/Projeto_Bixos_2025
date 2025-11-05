#include "h_bridge.h"
#include "encoder.h"
#include "PID.h"
#include "uart_esp32.h"
#include "pid_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Filas para comunicação entre tasks
QueueHandle_t rads_queue;
QueueHandle_t target_rads_queue;

// Handles dos encoders
pcnt_unit_handle_t left_encoder;
pcnt_unit_handle_t right_encoder;

// Variáveis globais
float valpidR = 0, valpidL = 0;
const char* TAG_MAIN = "MAIN_ROBOT";

void init_all()
{
    ESP_LOGI(TAG_MAIN, "Inicializando hardware...");
    
    init_gpio(LEFT_MOTOR);
    init_gpio(RIGHT_MOTOR);

    init_pwm(LEFT_MOTOR);
    init_pwm(RIGHT_MOTOR);

    left_encoder = init_encoder(ENCODER_LEFT);
    right_encoder = init_encoder(ENCODER_RIGHT);

    init_uart_read();
    init_uart_write();
    
    ESP_LOGI(TAG_MAIN, "Hardware inicializado!");
}

// Task 1: Comunicação UART com ROS
void uart_communication_task(void *pvParameters)
{
    target_rads_data_t target_rads;
    target_rads_data_t last_target_rads = {0, 0};
    rads_data_t current_rads;
    
    ESP_LOGI(TAG_MAIN, "Task UART iniciada");
    
    while(1) {
        // 1. Receber comandos do ROS
        target_rads = receive_data(&last_target_rads);
        
        // Se recebeu novo comando, enviar para a fila
        if (target_rads.target_left_rads != last_target_rads.target_left_rads ||
            target_rads.target_right_rads != last_target_rads.target_right_rads) {
            
            xQueueSend(target_rads_queue, &target_rads, 0);
            ESP_LOGI(TAG_MAIN, "Comando ROS: L=%.3f, R=%.3f", 
                    target_rads.target_left_rads, target_rads.target_right_rads);
        }
        last_target_rads = target_rads;
        
        // 2. Coletar dados atuais para enviar ao ROS - USANDO SUAS FUNÇÕES ATUAIS
        float conversion_rate = 0.00475;
        current_rads.left_rads = pulse_count(left_encoder) * conversion_rate * 20.0; // rad/s
        current_rads.right_rads = pulse_count(right_encoder) * conversion_rate * 20.0; // rad/s
        
        // 3. Enviar dados para ROS
        send_data(current_rads);
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}

// Task 2: Controle PID dos Motores
void motor_control_task(void *pvParameters)
{
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);
    target_rads_data_t target_rads;

    ESP_LOGI(TAG_MAIN, "Task controle motor iniciada");
    
    while(1) {
        // Verificar se há novo comando na fila
        if(xQueueReceive(target_rads_queue, &target_rads, 0) == pdPASS) {
            // Calcular PID para ambos os motores - USANDO SUA FUNÇÃO pid_calculate ATUAL
            pid_calculate(left_pid, LEFT_MOTOR, target_rads.target_left_rads, &valpidL, left_encoder);
            pid_calculate(right_pid, RIGHT_MOTOR, target_rads.target_right_rads, &valpidR, right_encoder);
            
            // A função pid_calculate já chama update_motor internamente!
            // Não precisa chamar set_motor_speed separadamente
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

// Task 3: Monitoramento (debug)
void monitoring_task(void *pvParameters)
{
    rads_data_t current_rads;
    target_rads_data_t target_rads;
    float conversion_rate = 0.00475;
    
    ESP_LOGI(TAG_MAIN, "Task monitoramento iniciada");
    
    while(1) {
        // Ler estado atual 
        current_rads.left_rads = pulse_count(left_encoder) * conversion_rate * 20.0;
        current_rads.right_rads = pulse_count(right_encoder) * conversion_rate * 20.0;
        
        // Tentar pegar último comando 
        if(xQueuePeek(target_rads_queue, &target_rads, 0) == pdPASS) {
            ESP_LOGI(TAG_MAIN, " Alvo: L=%.3f, R=%.3f | Medido: L=%.3f, R=%.3f | PID: L=%.1f, R=%.1f",
                    target_rads.target_left_rads, target_rads.target_right_rads,
                    current_rads.left_rads, current_rads.right_rads,
                    valpidL, valpidR);
        } else {
            ESP_LOGI(TAG_MAIN, " Medido: L=%.3f, R=%.3f | PID: L=%.1f, R=%.1f",
                    current_rads.left_rads, current_rads.right_rads,
                    valpidL, valpidR);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
    }
}

void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI(TAG_MAIN, "INICIANDO...");
    
    // 1. Inicializar hardware
    init_all();
    
    // 2. Criar filas
    rads_queue = xQueueCreate(10, sizeof(rads_data_t));
    target_rads_queue = xQueueCreate(10, sizeof(target_rads_data_t));
    
    if (rads_queue == NULL || target_rads_queue == NULL) {
        ESP_LOGE(TAG_MAIN, "Erro criando filas!");
        return;
    }
    
    // 3. Criar tasks
    xTaskCreatePinnedToCore(uart_communication_task, "uart_comm", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL, 4, NULL, 1);
    xTaskCreate(monitoring_task, "monitor", 4096, NULL, 1, NULL);
    
    ESP_LOGI(TAG_MAIN, "Sistema inicializado! Pronto para comunicação ROS.");
    ESP_LOGI(TAG_MAIN, "Aguardando comandos no /cmd_vel...");
    
    // Task principal fica vazia
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 segundos
    }
}
    
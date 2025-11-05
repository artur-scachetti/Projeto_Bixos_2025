#include "uart_esp32.h"
#include "h_bridge.h"
#include "PID.h"
#include "encoder.h"
#include "esp_log.h"

static const char *UART_TAG = "UART_ESP32";

void init_uart_read()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_READ, &uart_config);
    uart_set_pin(UART_PORT_NUM_READ, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_READ, BUFFER_LEN, 0, 0, 0, 0);
    
    ESP_LOGI(UART_TAG, "UART read inicializada");
}

target_rads_data_t receive_data(target_rads_data_t *last_target_rads)
{
    target_rads_data_t target_rads = *last_target_rads; // Usar último valor como default

    uint8_t buffer[BUFFER_LEN];
    int len = uart_read_bytes(UART_PORT_NUM_READ, buffer, BUFFER_LEN - 1, pdMS_TO_TICKS(50));

    if(len > 0)
    {
        buffer[len] = '\0';
        
        // DEBUG: Mostrar dados brutos recebidos
        ESP_LOGI(UART_TAG, "Recebido RAW: %s", buffer);

        char *token = strtok((char *)buffer, ";");
        if(token != NULL)
        {
            target_rads.target_left_rads = atof(token);
            ESP_LOGI(UART_TAG, "Left convertido: %f", target_rads.target_left_rads);
        }
        
        token = strtok(NULL, ";");
        if(token != NULL)
        {
            target_rads.target_right_rads = atof(token);
            ESP_LOGI(UART_TAG, "Right convertido: %f", target_rads.target_right_rads);
        }

        *last_target_rads = target_rads;
        ESP_LOGI(UART_TAG, "Comando processado: L=%.3f, R=%.3f", 
                target_rads.target_left_rads, target_rads.target_right_rads);
    }

    return target_rads;
}

void init_uart_write()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_WRITE, &uart_config);
    uart_set_pin(UART_PORT_NUM_WRITE, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_WRITE, BUFFER_LEN, 0, 0, 0, 0);
    
    ESP_LOGI(UART_TAG, "UART write inicializada");
}

void send_data(rads_data_t rads)
{
    char buffer[BUFFER_LEN];
    
    // CORREÇÃO CRÍTICA: usar strlen em vez de BUFFER_LEN
    int length = snprintf(buffer, sizeof(buffer), "%.3f;%.3f\n", 
                         rads.left_rads, rads.right_rads);
    
    if (length > 0 && length < sizeof(buffer)) {
        uart_write_bytes(UART_PORT_NUM_WRITE, buffer, length);
        ESP_LOGI(UART_TAG, "Enviado: %s", buffer); // Já inclui \n
    } else {
        ESP_LOGE(UART_TAG, "Erro ao formatar dados para envio");
    }
}
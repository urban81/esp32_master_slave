#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h" // Нужен для точного контроля таймаута в 1 сек

// --- НАСТРОЙКИ ЖЕЛЕЗА ДЛЯ ESP32-WROOM ---
// Настройки железа для ESP32-WROOM (ИСПРАВЛЕНО: Безопасные пины)
#define ROLE_PIN            GPIO_NUM_4
#define UART_PORT_NUM       UART_NUM_2
#define TXD_PIN             GPIO_NUM_17  // Вместо 17 ставим 22
#define RXD_PIN             GPIO_NUM_16  // Вместо 16 ставим 23
#define RX_BUF_SIZE         1024

// --- АДРЕСА УСТРОЙСТВ В ПРОТОКОЛЕ ---
#define MASTER_ADDRESS      0x01
#define SLAVE_ADDRESS       0x02

#define MAX_DATA_SIZE       255
#define MAX_PACKET_SIZE     (4 + 1 + 1 + 1 + MAX_DATA_SIZE + 1) // 263 байта всего

// Перечисление для ролей
typedef enum {
    ROLE_SLAVE = 0,
    ROLE_MASTER = 1
} DeviceRole_t;

// Состояния конечного автомата для приема
typedef enum {
    STATE_PREAMBLE_0,
    STATE_PREAMBLE_1,
    STATE_PREAMBLE_2,
    STATE_PREAMBLE_3,
    STATE_LENGTH,
    STATE_SRC,
    STATE_DST,
    STATE_DATA,
    STATE_CRC
} RxState_t;

// Задание 1. Структура сообщения
typedef struct {
    uint8_t preamble; // В коде проверим все 4 байта, в структуру пишем признак
    uint8_t length;
    uint8_t src_addr;
    uint8_t dst_addr;
    uint8_t data[MAX_DATA_SIZE];
    uint8_t crc;
} Packet_t;

// Глобальные переменные программы
static DeviceRole_t current_role = ROLE_SLAVE;
static const char *TAG = "PROTOCOL_APP";
// Функция определения роли по ножке GPIO4
void init_role_pin(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROLE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Если ножка в воздухе — это SLAVE (0)
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(50)); // пауза на стабилизацию уровня
    
    // Читаем физический уровень на ножке
 if (gpio_get_level(ROLE_PIN) == 1) {
    current_role = ROLE_MASTER;
    ESP_LOGI(TAG, "Device role determined as: MASTER");
} else {
    current_role = ROLE_SLAVE;
    ESP_LOGI(TAG, "Device role determined as: SLAVE");
}

}

// Настройка аппаратного UART2
void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Установка драйвера буферов (без очереди событий)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // Привязываем физические ножки: TX = GPIO17, RX = GPIO16
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

uint8_t Protocol_CalculateCRC(const uint8_t *buffer, uint16_t size) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < size; i++) {
        crc ^= buffer[i];
    }
    return crc;
}

// --- Задание 2. Реализация алгоритма приема (Конечный автомат) ---
bool Protocol_ReceivePacket(Packet_t *packet) {
    static RxState_t state = STATE_PREAMBLE_0;
    static int64_t start_time_us = 0; // Время старта приема пакета в микросекундах
    static uint16_t data_counter = 0;
    
    static uint8_t raw_buffer[MAX_PACKET_SIZE];
    static uint16_t raw_index = 0;

    // Проверка таймаута: 1 секунда = 1 000 000 микросекунд
    if (state != STATE_PREAMBLE_0 && (esp_timer_get_time() - start_time_us > 1000000)) {
        ESP_LOGW(TAG, "Message reception timeout (exceeded 1 second). Buffer flush.");
        state = STATE_PREAMBLE_0;
        raw_index = 0;
    }

    uint8_t byte_in;
    // Побайтовое чтение из буфера UART без блокировки (wait_ticks = 0)
    while (uart_read_bytes(UART_PORT_NUM, &byte_in, 1, 0) > 0) {
        
        // Как только поймали первый байт преамбулы — запускаем отсчет 1 секунды
        if (state == STATE_PREAMBLE_0 && byte_in == 0x01) {
            start_time_us = esp_timer_get_time();
            raw_index = 0;
        }

        // Защита от переполнения нашего буфера
        if (raw_index < MAX_PACKET_SIZE) {
            raw_buffer[raw_index++] = byte_in;
        } else {
            state = STATE_PREAMBLE_0;
            return false;
        }

        // Логика разбора по состояниям
        switch (state) {
            case STATE_PREAMBLE_0:
                state = (byte_in == 0x01) ? STATE_PREAMBLE_1 : STATE_PREAMBLE_0;
                break;
            case STATE_PREAMBLE_1:
                state = (byte_in == 0x02) ? STATE_PREAMBLE_2 : STATE_PREAMBLE_0;
                break;
            case STATE_PREAMBLE_2:
                state = (byte_in == 0x03) ? STATE_PREAMBLE_3 : STATE_PREAMBLE_0;
                break;
            case STATE_PREAMBLE_3:
                if (byte_in == 0x04) {
                    packet->preamble = 0x01; // Условная запись признака преамбулы
                    state = STATE_LENGTH;
                } else {
                    state = STATE_PREAMBLE_0;
                }
                break;
            case STATE_LENGTH:
                packet->length = byte_in;
                state = STATE_SRC;
                break;
            case STATE_SRC:
                packet->src_addr = byte_in;
                state = STATE_DST;
                break;
            case STATE_DST:
                packet->dst_addr = byte_in;
                data_counter = 0;
                // Если длина данных равна 0, то сразу переходим к проверке CRC
                state = (packet->length == 0) ? STATE_CRC : STATE_DATA;
                break;
            case STATE_DATA:
                packet->data[data_counter++] = byte_in;
                if (data_counter >= packet->length) {
                    state = STATE_CRC;
                }
                break;
            case STATE_CRC:
                packet->crc = byte_in;
                state = STATE_PREAMBLE_0; // Сбрасываем автомат для следующего кадра

                // Валидация контрольной суммы пакета.
                // Вычитаем 1 из raw_index, так как сам байт CRC в расчет XOR не входит
                uint8_t calc_crc = Protocol_CalculateCRC(raw_buffer, raw_index - 1);
                if (calc_crc == packet->crc) {
                    return true; // ПАКЕТ УСПЕШНО ПРИНЯТ И ПРОВЕРЕН!
                } else {
                    ESP_LOGE(TAG, "ERR CRC! CALC: 0x%02X, in paket: 0x%02X", calc_crc, packet->crc);
                    return false;
                }
                break;
        }
    }
    return false; // Пакет еще собирается, ждем новые байты
}

// Функция для формирования и отправки пакета в UART2
void Protocol_SendPacket(uint8_t dst_addr, const uint8_t *data, uint8_t length) {
    uint8_t tx_buffer[MAX_PACKET_SIZE];
    
    // 1. Преамбула (4 байта)
    tx_buffer[0] = 0x01;
    tx_buffer[1] = 0x02;
    tx_buffer[2] = 0x03;
    tx_buffer[3] = 0x04;
    
    // 2. Длина данных
    tx_buffer[4] = length;
    
    // 3. Адрес источника (смотрим на нашу текущую роль)
    tx_buffer[5] = (current_role == ROLE_MASTER) ? MASTER_ADDRESS : SLAVE_ADDRESS;
    
    // 4. Адрес приёмника
    tx_buffer[6] = dst_addr;
    
    // 5. Копируем полезные данные
    if (length > 0 && data != NULL) {
        memcpy(&tx_buffer[7], data, length);
    }
    
    // 6. Считаем CRC всех байт (7 байт заголовка + длина данных)
    uint16_t bytes_to_calculate = 7 + length;
    tx_buffer[bytes_to_calculate] = Protocol_CalculateCRC(tx_buffer, bytes_to_calculate);
    
    // 7. Отправляем готовый массив байт в UART2
    uint16_t total_packet_size = bytes_to_calculate + 1;
    uart_write_bytes(UART_PORT_NUM, (const char*)tx_buffer, total_packet_size);
}

// --- Шаг 4. Логика работы устройства во FreeRTOS ---
void protocol_task(void *pvParameters) {
    Packet_t rx_packet;
    uint32_t tick_count = 0;

    while (1) {
        // 1. Постоянный опрос приемника UART2
        if (Protocol_ReceivePacket(&rx_packet)) {
            ESP_LOGI(TAG, "PACKET RECEIVED SUCCESSFULLY! From: 0x%02X, To: 0x%02X, Data Length: %d", 
                     rx_packet.src_addr, rx_packet.dst_addr, rx_packet.length);
            
            // Если устройство работает как СЛЕЙВ и пакет пришел для него (адрес 0x02)
            if (current_role == ROLE_SLAVE && rx_packet.dst_addr == SLAVE_ADDRESS) {
                vTaskDelay(pdMS_TO_TICKS(50)); // Небольшая пауза перед ответом
                
                // Формируем эхо-ответ для Мастера
                const char *reply_msg = "HELLO MASTER";
                Protocol_SendPacket(MASTER_ADDRESS, (const uint8_t*)reply_msg, strlen(reply_msg));
                ESP_LOGI(TAG, "The Slave sent a reply to the Master.");
            }
        }

        // 2. Логика отправки для МАСТЕРА (отсылает PING каждые 2 секунды)
        if (current_role == ROLE_MASTER) {
            tick_count++;
            if (tick_count >= 200) { // 200 циклов * 10мс задержки = 2000мс (2 секунды)
                tick_count = 0;
                
                const char *ping_msg = "PING";
                ESP_LOGI(TAG, "Master initiates PING...");
                Protocol_SendPacket(SLAVE_ADDRESS, (const uint8_t*)ping_msg, strlen(ping_msg));
            }
        }

        // Обязательная задержка FreeRTOS на 10 миллисекунд, чтобы дать процессору "подышать"
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// --- Главная точка входа в программу ESP-IDF ---
void app_main(void) {
    // 1. Сначала определяем роль устройства по ножке GPIO4
    init_role_pin();
    
    // 2. Настраиваем аппаратный порт UART2 на нужные пины
    init_uart();

    // 3. Создаем и запускаем фоновую задачу во FreeRTOS
    // Имя задачи: "protocol_task", размер стека: 4096 байт, приоритет: 5
    xTaskCreate(protocol_task, "protocol_task", 4096, NULL, 5, NULL);
}


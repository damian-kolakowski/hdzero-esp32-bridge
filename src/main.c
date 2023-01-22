#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#define BRIDGE_AP_SSID          ("HDZERO")
#define BRIDGE_SOCKET_PORT_NUM  (8080)
#define BRIDGE_UART_PORT_NUM    (UART_NUM_0)
#define BRIDGE_UART_BAUD        (115200)
#define BRIDGE_TASK_STACK_SIZE  (2048)
#define BRIDGE_BUF_SIZE         (512)
#define BRIDGE_READY_PREAMBLE   ("HDZERO READY\r\n")

int client_socket = -1;

static void uart_print(const char* data) {
    uart_write_bytes(BRIDGE_UART_PORT_NUM, data, strlen(data));
}

static void uart_read_task(void *arg)
{
    char data[BRIDGE_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(BRIDGE_UART_PORT_NUM, data, BRIDGE_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (client_socket != -1 && len > 0) {
            int to_write = len;
            while (to_write > 0) {  
                int written = send(client_socket, data + (len - to_write), to_write, 0);
                if (written < 0) {
                    uart_print("Error occurred during sending: errnor \r\n");
                    return;
                }
                to_write -= written;
            }
        }
    }
}

static void do_retransmit(const int sock)
{
    int len;
    char buffer[BRIDGE_BUF_SIZE];
    do {
        len = recv(sock, buffer, sizeof(buffer), 0);
        if (len < 0) {
            uart_print("Error occurred during receiving: errno\r\n");
        } else if (len == 0) {
            uart_print("Connection closedr\r\n");
        } else {
            uart_print("Received bytes: \r\n");
            int to_write = len;
            while (to_write > 0) {  
                int written = uart_write_bytes(BRIDGE_UART_PORT_NUM, buffer + (len - to_write), to_write);
                if (written < 0) {
                    uart_print("Error occurred during sending: errnor \r\n");
                    return;
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 30;
    int keepCount = 3;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(BRIDGE_SOCKET_PORT_NUM);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        uart_print("Unable to create socket: errno \r\n");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;

    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    uart_print("Socket created\r\n");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        uart_print("Socket unable to bind: errno \r\n");
        goto CLEAN_UP;
    }
    uart_print("Socket bound, port ");

    err = listen(listen_sock, 1);
    if (err != 0) {
        uart_print("Error occurred during listen: errno \r\n");
        goto CLEAN_UP;
    }

    while (1) {

        uart_print("Waiting for incoming connection...\r\n");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        if (sock < 0) {
            uart_print("Unable to accept connection.\r\n");
            break;
        }

        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
    
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        uart_print(BRIDGE_READY_PREAMBLE);

        client_socket = sock;
        
        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
       // wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
       // wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = BRIDGE_AP_SSID,
            .ssid_len = strlen(BRIDGE_AP_SSID),
            .channel = 1,
            .password = "",
            .max_connection = 5,
            .authmode = WIFI_AUTH_OPEN
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    uart_print("wifi_init_softap done.\r\n");
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = BRIDGE_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_PORT_NUM, BRIDGE_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_PORT_NUM, &uart_config));
    
    ESP_ERROR_CHECK(uart_set_pin(
        BRIDGE_UART_PORT_NUM, 
        UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE
    ));

    uart_print("uart_driver_install done.\r\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uart_print("nvs_flash_init done.\r\n");

    wifi_init_softap();

    uart_print("runnin tasks....\r\n");

    xTaskCreate(uart_read_task, "uart_read_task", BRIDGE_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(tcp_server_task, "tcp_server_task", BRIDGE_TASK_STACK_SIZE, (void*)AF_INET, 10, NULL);
}

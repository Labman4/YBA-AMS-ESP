/* Wi-Fi Provisioning Manager Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>


#include "esp_system.h"
#include "esp_netif.h"

#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdlib.h>
#include "esp_eap_client.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"
#define CONFIG_EXAMPLE_IPV4
#define CONFIG_EXAMPLE_IPV6
#define CONFIG_UDP_SERVER
#define PORT 3333
#define MY_ADDR 0

#define M1_UP 0
#define M1_DOWN 1
#define M2_UP 18
#define M2_DOWN 19
#define M3_UP 2
#define M3_DOWN 3
#define M4_UP 10
#define M4_DOWN 6
#define SW1 7
#define SW2 5
#define SW3 4
#define SW4 11

#define LED1 12
// #define LED2 13

#define GPIO_OUTPUT_PIN_SEL ((1ULL << M1_UP) |   \
                             (1ULL << M1_DOWN) | \
                             (1ULL << M2_UP) |   \
                             (1ULL << M2_DOWN) | \
                             (1ULL << M3_UP) |   \
                             (1ULL << M3_DOWN) | \
                             (1ULL << M4_UP) |   \
                             (1ULL << M4_DOWN) | \
                             (1ULL << LED1))

#define GPIO_INPUT_PIN_SEL ((1ULL << SW1) | \
                            (1ULL << SW2) | \
                            (1ULL << SW3) | \
                            (1ULL << SW4))

#define ATTR_NO_ALIGN __attribute__((packed))

typedef enum
{
    STATUS,           /**< 状态信息 */
    DC_MOTOR_CONTROL, /**< 直流电机控制 */
    PARAM_SET,        /**< 参数设置 */
} yba_ams_cmd_e;

typedef enum
{
    STOP,    /**< 停止 */
    FORWARD, /**< 送料 */
    REVERSE, /**< 退料 */
} yba_ams_dc_fx_e;

typedef struct
{
    uint16_t head;     /**< 包头 */
    uint8_t dest_addr; /**< 目标地址 */
    uint8_t src_addr;  /**< 源地址 */
    uint8_t cmd;       /**< 命令 */
    uint8_t len;       /**< 数据长度 */
    uint8_t data[0];   /**< 数据指针 */
} ATTR_NO_ALIGN yba_ams_frame_format_t;

/** 直流电机控制 */
typedef struct
{
    uint8_t id;          /**< 电机位号 */
    uint8_t fx;          /**< 方向 */
    uint8_t duty;        /**< 占空百分比 */
    uint32_t timeout;    /**< 超时时间ms */
    uint32_t shift_time; /**< 变速时间ms */
} ATTR_NO_ALIGN yba_ams_dc_t;

int ch_io[4][4] = {
    {M1_UP, M1_DOWN},
    {M2_UP, M2_DOWN},
    {M3_UP, M3_DOWN},
    {M4_UP, M4_DOWN}};

int sw_io[4] = {SW1, SW2, SW3, SW4};

static const char *TAG = "app";
static const char *SMART_TAG = "smartConfig";

// 各个通道当前状态
int channel_status[4] = {0, 0, 0, 0};
int switch_status[4] = {0, 0, 0, 0};

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
#define NVS_NAMESPACE "wifi_config"


static void smartconfig_example_task(void * parm);
static void save_wifi_config(const char* ssid, const char* pass);

void udp_broadcast_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(9999);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
    }
    unsigned char mac_base[6] = {0};
    esp_err_t err = esp_efuse_mac_get_default(mac_base);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get default MAC address");
    } else {
        char mac_str[18];
        sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
        ESP_LOGI(TAG, "MAC address: %s", mac_str);
        while (1) {
            if (sendto(sock, mac_str, strlen(mac_str), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
                ESP_LOGE(TAG, "Failed to send broadcast");
            }
            ESP_LOGI(TAG, "send broadcast");
            vTaskDelay(pdMS_TO_TICKS(10000)); // Send broadcast every second
        }
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(SMART_TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(SMART_TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(SMART_TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        char ssid[33] = { 0 };
        char password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            ESP_LOGI(SMART_TAG, "Set MAC address of target AP: "MACSTR" ", MAC2STR(evt->bssid));
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
#endif

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(SMART_TAG, "SSID:%s", ssid);
        ESP_LOGI(SMART_TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(SMART_TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
        save_wifi_config(ssid, password);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void save_wifi_config(const char* ssid, const char* pass) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_flash_init();
    if (err != ESP_OK) {
        return;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }

    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err != ESP_OK) {
        return;
    }
    err = nvs_set_str(nvs_handle, "pass", pass);
    if (err != ESP_OK) {
        return;
    }

    nvs_close(nvs_handle);
}

static void initialise_wifi(void)
{   
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
   
}

static bool load_wifi_config_and_connect() {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    ESP_LOGI(TAG, "try to load wifi info");

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }

    char ssid[32], password[64];
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);
    err = nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "load wifi info %s", ssid);
        err = nvs_get_str(nvs_handle, "pass", password, &pass_len);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "load wifi info %s", password);
            wifi_config_t wifi_config;
            memset(&wifi_config, 0, sizeof(wifi_config_t));
            strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
            strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
            wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
            wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0'; 
            esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
            ESP_LOGI(TAG, "connect wifi info");
            ESP_LOGI(TAG, "load wifi ssid %s",  wifi_config.sta.ssid);
            ESP_LOGI(TAG, "load wifi pw %s",  wifi_config.sta.password);
            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            esp_wifi_connect();
            return true;
        }
    } else {
        ESP_LOGI(TAG, "wait for wifi info");
    }
    return false;
}


static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(SMART_TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(SMART_TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

void motor_control(int id, int fx) // , int duty, int timeout, int shift_time)
{
    switch (fx)
    {
    case REVERSE:
        channel_status[id] = REVERSE;
        gpio_set_level(ch_io[id][0], 1);
        gpio_set_level(ch_io[id][1], 0);
        break;
    case FORWARD:
        channel_status[id] = FORWARD;
        gpio_set_level(ch_io[id][0], 0);
        gpio_set_level(ch_io[id][1], 1);
        break;
    case STOP:
        channel_status[id] = STOP;
        gpio_set_level(ch_io[id][0], 0);
        gpio_set_level(ch_io[id][1], 0);
        break;
    }
}

void sock_send(int sock, uint8_t cmd, const char *data, int len)
{
    yba_ams_frame_format_t *frame = malloc(sizeof(yba_ams_frame_format_t) + len);
    frame->head = 0x2F2F;
    frame->dest_addr = 0xFE;
    frame->src_addr = MY_ADDR;
    frame->cmd = cmd;
    frame->len = len;
    memcpy(frame->data, data, len);
    send(sock, (const char *)frame, sizeof(yba_ams_frame_format_t) + len, 0);
}

static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];

    do
    {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        ESP_LOGI(TAG, "Received %d bytes", len);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
            break;
        }
        else if (len > 2)
        {
            gpio_set_level(LED1, 0);
            // int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
            yba_ams_frame_format_t *frame = (yba_ams_frame_format_t *)rx_buffer;
            if (frame->head == 0x2F2F)
            {
                if (frame->dest_addr != MY_ADDR && frame->dest_addr != 0xFF)
                {
                    continue;
                }
                switch (frame->cmd)
                {
                case DC_MOTOR_CONTROL:
                {
                    yba_ams_dc_t *dc = (yba_ams_dc_t *)frame->data;
                    int id = dc->id - MY_ADDR * 4;
                    if (id < 0 || id >= 4)
                    {
                        break;
                    }
                    motor_control(id, dc->fx);

                    sock_send(sock, STATUS, NULL, 0);
                }
                break;
                case PARAM_SET:
                    break;
                default:
                    break;
                }
            }
            gpio_set_level(LED1, 1);
        } else {
            ESP_LOGI(TAG, "Received %d bytes", len);
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
    int keepInterval = 5;
    int keepCount = 3;
    struct sockaddr_storage dest_addr;

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    if (addr_family == AF_INET6)
    {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 4);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
        if (source_addr.ss_family == PF_INET6)
        {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);
        for (int i = 0; i < 4; i++)
        {
            motor_control(i, 0);
        }
        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void GPIO_INIT()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    for (int i = 0; i < 4; i++)
    {
        gpio_set_level(ch_io[i][0], 0);
        gpio_set_level(ch_io[i][1], 0);
    }

    gpio_set_level(LED1, 1);
}

void app_main(void)
{
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI(TAG, "nvs clean ");
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    initialise_wifi();
    if (!load_wifi_config_and_connect()) {
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } 
    GPIO_INIT();

    for (int i = 0; i < 4; i++)
    {
        gpio_set_level(ch_io[i][0], 0);
        gpio_set_level(ch_io[i][1], 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(ch_io[i][0], 1);
        gpio_set_level(ch_io[i][1], 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(ch_io[i][0], 0);
        gpio_set_level(ch_io[i][1], 0);
    }

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());


#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET6, 5, NULL);
#endif
#ifdef CONFIG_UDP_SERVER
    xTaskCreate(&udp_broadcast_task, "udp_broadcast_task", 2048, NULL, 5, NULL);
#endif 
    ESP_LOGI(TAG, "hi!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            if (channel_status[i] == REVERSE)
            {
                if (gpio_get_level(sw_io[i]) == 1)
                {
                    gpio_set_level(ch_io[i][0], 0);
                    gpio_set_level(ch_io[i][1], 0);
                }
                else
                {
                    gpio_set_level(ch_io[i][0], 1);
                    gpio_set_level(ch_io[i][1], 0);
                }
            }
            else if (i == 3) {
                // if (gpio_get_level(sw_io[i] == 0)) {
                //     gpio_set_level(LED1, 1);
                // }                                                                                                                                                                                                                                         
                // ESP_LOGI(TAG, "sw ! %d", sw_io[i]);
                // ESP_LOGI(TAG, "sw io ! %d", gpio_get_level(sw_io[i]));
                if (gpio_get_level(sw_io[i]) == 0)
                {
                    switch_status[i] = 5;
                    gpio_set_level(ch_io[i][0], 0);
                    gpio_set_level(ch_io[i][1], 1);
                }
                else if (switch_status[i] > 0)
                {
                    if (switch_status[i] == 1)
                    {
                        motor_control(i, channel_status[i]);
                    }
                    switch_status[i]--;
                }
            } else
            {
                if (gpio_get_level(sw_io[i]) == 1)
                {
                    switch_status[i] = 5;
                    gpio_set_level(ch_io[i][0], 0);
                    gpio_set_level(ch_io[i][1], 1);
                }
                else if (switch_status[i] > 0)
                {
                    if (switch_status[i] == 1)
                    {
                        motor_control(i, channel_status[i]);
                    }
                    switch_status[i]--;
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <components/homekit/include/homekit/types.h>
#include "wifi.h"
#include "dht11.h"

#define GPIO_IDENTITY_LED GPIO_NUM_18
#define GPIO_ACTION_LED GPIO_NUM_19
#define GPIO_BUTTON GPIO_NUM_17
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_HUMIDITY_TEMPERATURE GPIO_NUM_17
#define GPIO_DHT GPIO_NUM_4
#define GPIO_RESET_BUTTON GPIO_NUM_22
#define GPIO_RESET_LED GPIO_NUM_2

static bool DOOR_IS_OPEN = false;
static bool RESET_PRESSED = false;

static homekit_value_t sensor_state_value = HOMEKIT_UINT8(0);
static homekit_value_t temperature_value = HOMEKIT_FLOAT(0);
static homekit_value_t status_active_value = HOMEKIT_BOOL(false);

void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("STA start\n");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("WiFI ready\n");
            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            printf("STA disconnected\n");
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

homekit_value_t sensor_state_getter() {
    printf("sensor_state_getter\n");

    return sensor_state_value;
}
homekit_value_t temperature_getter() {
    printf("temperature_getter\n");

    return temperature_value;
}
homekit_value_t status_active_getter() {
    printf("status_active_getter\n");

    return status_active_value;
}

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

homekit_characteristic_t sensor_state = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0, .getter=sensor_state_getter);
homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0, .getter=temperature_getter);
homekit_characteristic_t status_active = HOMEKIT_CHARACTERISTIC_(STATUS_ACTIVE, 0, .getter=status_active_getter);

static void gpio_watcher(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == GPIO_RESET_BUTTON) {
                if (gpio_get_level(io_num) == 1) {
                    if (RESET_PRESSED == false) {
                        RESET_PRESSED = true;
                        printf("Resetting Homekit...\n");
                        for (int i = 1; i <= 10; ++i) {
                            gpio_set_level(GPIO_RESET_LED, i % 2);
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                        }
                        gpio_set_level(GPIO_RESET_LED, false);
                        homekit_server_reset();
                        printf("Homekit server reset!\n");
                    }
                } else {
                    if (RESET_PRESSED == true) {
                        RESET_PRESSED = false;
                    }
                }
            } else {
                if (gpio_get_level(io_num) == 1) {
                    if (DOOR_IS_OPEN == false) {
                        DOOR_IS_OPEN = true;
                        sensor_state_value = HOMEKIT_UINT8(1);
                        printf("The door is open\n");
                        gpio_set_level(GPIO_ACTION_LED, true);
                        homekit_characteristic_notify(&sensor_state, HOMEKIT_UINT8(1));
                    }
                } else {
                    if (DOOR_IS_OPEN == true) {
                        DOOR_IS_OPEN = false;
                        sensor_state_value = HOMEKIT_UINT8(0);
                        printf("The door is closed\n");
                        gpio_set_level(GPIO_ACTION_LED, false);
                        homekit_characteristic_notify(&sensor_state, HOMEKIT_UINT8(0));
                    }
                }
            }
        }
    }
}

void reset_door_state()
{
    if (gpio_get_level(GPIO_BUTTON) == 1) {
        sensor_state_value = HOMEKIT_UINT8(1);
        printf("The door is open\n");
        gpio_set_level(GPIO_ACTION_LED, true);
    } else {
        sensor_state_value = HOMEKIT_UINT8(0);
        printf("The door is closed\n");
        gpio_set_level(GPIO_ACTION_LED, false);
    }
}

void led_write(bool on) {
    gpio_set_level(GPIO_IDENTITY_LED, on ? 1 : 0);
}

void led_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void led_init() {
    gpio_set_direction(GPIO_IDENTITY_LED, GPIO_MODE_OUTPUT);
    led_write(false);
}

void led_identify(homekit_value_t _value) {
    printf("LED identify\n");
}

homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Ruud"),
                HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1337"),
                HOMEKIT_CHARACTERISTIC(MODEL, "MyLED"),
                HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
                HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
                NULL
                }),
              HOMEKIT_SERVICE(CONTACT_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
                      HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                      &sensor_state,
                      NULL
              }),
              NULL
        }),
        HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
                HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Ruud"),
                HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1337"),
                HOMEKIT_CHARACTERISTIC(MODEL, "MyLED"),
                HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
                HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
                NULL
                }),
              HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
                      HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
                      &current_temperature,
                      &status_active,
                      NULL
              }),
              NULL
        }),
        NULL
};

homekit_server_config_t config = {
        .accessories = accessories,
        .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}


void DHT_task(void *pvParameter)
{
    printf("Waiting 1 second to start DHT measurement...\n");
    vTaskDelay(1000 / portTICK_RATE_MS);

    printf("Starting DHT measurement!\n");
    while(1)
    {
        int temp = getTemp();

        if (temp == DHT_CHECKSUM_ERROR) {
            printf("Checksum error, skipping\n");

            status_active_value.bool_value = false;
            homekit_characteristic_notify(&status_active, status_active_value);
        } else if (temp == DHT_TIMEOUT_ERROR) {
            printf("Timeout error, skipping\n");

            status_active_value.bool_value = false;
            homekit_characteristic_notify(&status_active, status_active_value);
        } else {
            printf("Temperature reading %d\n",temp);

            temperature_value.float_value = (float) temp;
            homekit_characteristic_notify(&current_temperature, temperature_value);

            status_active_value.bool_value = true;
            homekit_characteristic_notify(&status_active, status_active_value);
        }

        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

void app_main(void) {
    uart_set_baudrate(0, 115200);

    gpio_set_direction(GPIO_ACTION_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_ANYEDGE);
    gpio_set_pull_mode(GPIO_BUTTON, GPIO_PULLUP_ONLY);

    gpio_set_direction(GPIO_RESET_BUTTON, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_RESET_BUTTON, GPIO_INTR_ANYEDGE);
    gpio_set_pull_mode(GPIO_RESET_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_direction(GPIO_RESET_LED, GPIO_MODE_OUTPUT);

    reset_door_state();

    printf("App main\n");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    printf("Init wifi\n");
    wifi_init();
    led_init();

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(gpio_watcher, "gpio_watcher", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);
    gpio_isr_handler_add(GPIO_RESET_BUTTON, gpio_isr_handler, (void*) GPIO_RESET_BUTTON);

    xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
}

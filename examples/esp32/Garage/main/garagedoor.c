#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi.h"

#include "freertos/timers.h"

#include <freertos/queue.h>
#include <freertos/task.h>

#define TIMERTAG "TIMER"

// Possible values for characteristic CURRENT_DOOR_STATE:
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING 2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING 3
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED 4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN 255

const char *state_description(uint8_t state) {
    const char* description = "unknown";
    switch (state) {
        case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN: description = "open"; break;
        case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: description = "opening"; break;
        case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED: description = "closed"; break;
        case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: description = "closing"; break;
        case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED: description = "stopped"; break;
        default: ;
    }
    return description;
}

#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN 0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED 1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN 255

#define OPEN_CLOSE_DURATION 15
static gpio_num_t RELAY_PIN = GPIO_NUM_2;
static gpio_num_t REED_PIN = 14;

//variables
bool relay_on = false;
uint8_t current_door_state = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN;

bool obstruction = false;

// Declare global variables for reed sensor:
typedef enum {
    CONTACT_CLOSED,
    CONTACT_OPEN
} contact_sensor_state_t;



TimerHandle_t tmr;

static xQueueHandle gpio_evt_queue = NULL;



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
     printf("Try to connect to WIFI: %s\n", WIFI_SSID);
}

// Declare functions:

homekit_value_t gdo_target_state_get();
void gdo_target_state_set(homekit_value_t new_value);
homekit_value_t gdo_current_state_get();
homekit_value_t gdo_obstruction_get();
void identify(homekit_value_t _value);




homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_garage, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Garagentor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HD"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "237A2BAB119E"),
            HOMEKIT_CHARACTERISTIC(MODEL, "ESP32_GDO"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1.0"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
            NULL
        }),
        HOMEKIT_SERVICE(GARAGE_DOOR_OPENER, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Tor"),
            HOMEKIT_CHARACTERISTIC(
                                   CURRENT_DOOR_STATE, HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED,
                                   .getter=gdo_current_state_get,
                                   .setter=NULL
                                   ),
            HOMEKIT_CHARACTERISTIC(
                                   TARGET_DOOR_STATE, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED,
                                   .getter=gdo_target_state_get,
                                   .setter=gdo_target_state_set
                                   ),
            HOMEKIT_CHARACTERISTIC(
                                   OBSTRUCTION_DETECTED, HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED,
                                   .getter=gdo_obstruction_get,
                                   .setter=NULL
                                   ),
            NULL
        }),
        NULL
    }),
    NULL
};






void identify_task(void *_args) {
    // 1. move the door, 2. stop it, 3. move it back:
    for (int i=0; i<3; i++) {
        gpio_set_level(RELAY_PIN, 1);
        //printf("Relay ON\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(RELAY_PIN, 0);
        //printf("Relay OFF\n");
        vTaskDelay(3500 / portTICK_PERIOD_MS);
    }
    
    //relay_write(false);
    
    vTaskDelete(NULL);
}

void identify(homekit_value_t _value) {
   // printf("GDO identify\n");
   // xTaskCreate(identify_task, "GDO identify", 1024, NULL, 2, NULL);
}






homekit_value_t gdo_obstruction_get() {
    printf("returning obstruction state.\n");
    return HOMEKIT_BOOL(obstruction);
}

void gdo_current_state_notify_homekit() {
    
    homekit_value_t new_value = HOMEKIT_UINT8(current_door_state);
    printf("Notifying homekit that CURRENT DOOR STATE is now '%s'\n", state_description(current_door_state));
    
    // Find the current door state characteristic c:
    homekit_accessory_t *accessory = accessories[0];
    homekit_service_t *service = accessory->services[1];
    homekit_characteristic_t *c = service->characteristics[1];
    
    assert(c);
    
    //printf("Notifying changed '%s'\n", c->description);
    homekit_characteristic_notify(c, new_value);
}

void gdo_target_state_notify_homekit() {
    
    homekit_value_t new_value = gdo_target_state_get();
    printf("Notifying homekit that TARGET DOOR STATE is now '%s'\n", state_description(new_value.int_value));
    
    // Find the target door state characteristic c:
    homekit_accessory_t *accessory = accessories[0];
    homekit_service_t *service = accessory->services[1];
    homekit_characteristic_t *c = service->characteristics[2];
    
    assert(c);

    homekit_characteristic_notify(c, new_value);
}




void current_state_set(uint8_t new_state) {
    if (current_door_state != new_state) {
        current_door_state = new_state;
        gdo_target_state_notify_homekit();
        gdo_current_state_notify_homekit();
    }
}


contact_sensor_state_t contact_sensor_state_get(uint8_t gpio_num) {
    return gpio_get_level(gpio_num);
}


void current_door_state_update_from_sensor() {
    
    contact_sensor_state_t sensor_state = gpio_get_level(REED_PIN);
    printf("Contact sensor state'%d'......", sensor_state);
    switch (sensor_state) {
        case CONTACT_CLOSED:
            //printf("contact sensor event closed: %d\n", sensor_state);
            obstruction = false;
            current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED);
            break;
       
        case CONTACT_OPEN:
            //printf("contact sensor event open: %d\n", sensor_state);
            current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
            break;
       
        default:
            printf("Unknown contact sensor event: %d\n", sensor_state);
    }
}

homekit_value_t gdo_current_state_get() {
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN) {
        current_door_state_update_from_sensor();
    }
    printf("returning current door state '%s'.\n", state_description(current_door_state));
    
    return HOMEKIT_UINT8(current_door_state);
}

homekit_value_t gdo_target_state_get() {
    uint8_t result = gdo_current_state_get().int_value;
    if (result == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
        result = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } else if (result == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
        result = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }
    
    printf("Returning target door state '%s'.\n", state_description(result));
    
    return HOMEKIT_UINT8(result);
}

/**
 * Called from the interrupt handler to notify the client of a state change.
 **/
void contact_sensor_state_changed(uint8_t gpio, contact_sensor_state_t state) {
    
    printf("contact sensor state '%s'.\n", state == CONTACT_OPEN ? "open" : "closed");
    printf("current door state'%s'\n", state_description(current_door_state));
    
    if (current_door_state == HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED){
        current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
        ESP_LOGI(TIMERTAG,"Timer start because the door leave the closed position --- now opening");
            if( xTimerStart(tmr, 10 ) != pdPASS ) {
               ESP_LOGI(TIMERTAG,"Timer start error");
           }
        
    }
    else if (current_door_state != HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING)
    {
        current_door_state_update_from_sensor();
    }
    
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) {
        // Ignore the event - the state will be updated after the time expired!
        printf("contact_sensor_state_changed() ignored during opening.\n");
        return;
    }
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) {
        // Ignore the event - the state will be updated after the time expired!
        printf("contact_sensor_state_changed() ignored during closing.\n");
        return;
    }
}


void gdo_target_state_set(homekit_value_t new_value) {
    
    if (new_value.format != homekit_format_uint8) {
        printf("Invalid value format: %d\n", new_value.format);
        return;
    }
    
    if (current_door_state != HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN &&
        current_door_state != HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
        printf("gdo_target_state_set() ignored: current state not open or closed (%s).\n", state_description(current_door_state));
        return;
    }
    
    if (current_door_state == new_value.int_value) {
        printf("gdo_target_state_set() ignored: new target state == current state (%s)\n", state_description(current_door_state));
        return;
    }
    
    // Toggle the garage door by toggling the relay connected to the GPIO (on - off):
    // Turn ON GPIO:
    gpio_set_level(RELAY_PIN, 1);
    printf("Relay ON for 1sec\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(RELAY_PIN, 0);
    //printf("Relay OFF\n");
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) {
        current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING);
    } else {
        current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING);
    }
    // Wait for the garage door to open / close,
    // then update current_door_state from sensor:
    ESP_LOGI(TIMERTAG,"Timer start because new state set");
    if( xTimerStart(tmr, 10 ) != pdPASS ) {
        ESP_LOGI(TIMERTAG,"Timer start error");
    }
   
}



//timer


void timer_event_fired( TimerHandle_t xTimer )
{
    //please no printf here because of possible stack overflow
   // ESP_LOGI(TIMERTAG,"timer fired!!!");
    //printf("current DOOR state (%s)\n", state_description(current_door_state));
    
    
    if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING && contact_sensor_state_get(REED_PIN)==1) {
        //ESP_LOGI(TIMERTAG,"Garagedoor should be open now !!!");
        current_door_state_update_from_sensor();
        //current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN);
    }
    
     else if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING && contact_sensor_state_get(REED_PIN)==0){
        //ESP_LOGI(TIMERTAG,"Garagedoor obstruction detected in opening!!!");
         obstruction = true;
         current_door_state_update_from_sensor();
        //current_state_set(HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN);
        //OBSTRUCTION_DETECTED
    }
    
    else if (current_door_state == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING && contact_sensor_state_get(REED_PIN)==1){
    //ESP_LOGI(TIMERTAG,"Garagedoor obstruction detected in closing!!!");
    obstruction = true;
        current_door_state_update_from_sensor();
    
}
}

static bool paired = false;

void on_event(homekit_event_t event) {
    if (event == HOMEKIT_EVENT_SERVER_INITIALIZED) {
        //led_status_set(led_status, paired ? &normal_mode : &unpaired);
        printf("HOMEKIT_EVENT_SERVER_INITIALIZED\n");
    }
    else if (event == HOMEKIT_EVENT_CLIENT_CONNECTED) {
        if (!paired)
         //   led_status_set(led_status, &pairing);
            printf("HOMEKIT_EVENT_CLIENT_CONNECTED\n");
    }
    else if (event == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
        if (!paired)
          //  led_status_set(led_status, &unpaired);
            printf("HOMEKIT_EVENT_CLIENT_DISCONNECTED\n");
    }
    else if (event == HOMEKIT_EVENT_PAIRING_ADDED || event == HOMEKIT_EVENT_PAIRING_REMOVED) {
        paired = homekit_is_paired();
       // led_status_set(led_status, paired ? &normal_mode : &unpaired);
        printf("HOMEKIT_EVENT_PAIRING_ADDED ||  HOMEKIT_EVENT_PAIRING_REMOVED\n");
    }
}




homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111",
    .on_event = on_event,
};


void on_wifi_ready() {
    homekit_server_init(&config);
    printf("homekit server init: .....\n");
}



static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    //ets_printf("GPIO[%d] intr, val: %d\n", gpio_num, gpio_get_level(gpio_num));
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num = (uint32_t) arg;
    //ets_printf("task...GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
    static unsigned long last_interrupt_time = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            unsigned long interrupt_time = (xTaskGetTickCount() * portTICK_PERIOD_MS);
            //unsigned long interrupt_time = millis();
            // If interrupts come faster than 200ms, assume it's a bounce and ignore
            if (interrupt_time - last_interrupt_time > 300)
            {
                contact_sensor_state_changed(io_num,gpio_get_level(io_num));
            }
            last_interrupt_time = interrupt_time;
            
        }

    }
}




void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    
    //relay_init
    gpio_set_direction    (RELAY_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize Timer:
    int id = 1;
    int interval = OPEN_CLOSE_DURATION * 1000; //timer is in ms
    tmr = xTimerCreate("MyTimer", pdMS_TO_TICKS(interval), pdFALSE, ( void * )id, &timer_event_fired);
    
    
    //Configure sensor
    gpio_config_t btn_config;
    btn_config.intr_type = GPIO_INTR_ANYEDGE;     //Enable interrupt on both rising and falling edges
    btn_config.mode = GPIO_MODE_INPUT;            //Set as Input
    btn_config.pin_bit_mask = (1 << REED_PIN); //Bitmask
    btn_config.pull_up_en = GPIO_PULLUP_ENABLE;     //Disable pullup
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE; //Enable pulldown
    gpio_config(&btn_config);
    printf("Sensor configured from main\n");
    
    //Configure interrupt and add handler
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);                        //Start Interrupt Service Routine service
    gpio_isr_handler_add(REED_PIN, gpio_isr_handler, (void*) REED_PIN); //Add handler of interrupt
    printf("Interrupt configured from main\n");

    
    //xTaskCreate(&task_sensor_interrupt, "sensor_interrupt", 2048, NULL, 5, NULL);
    printf("Using Sensor at GPIO%d.\n", REED_PIN);

}

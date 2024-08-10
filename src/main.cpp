#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "LIDARLite_v3HP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "DFRobotDFPlayerMini.h"
#include <ttn_keys.h>
#include <lmic.h>
#include <hal/hal.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define LED 2
#define NUM_MEASUREMENTS 10
#define THRESHOLD_PERCENTAGE 0.05
#define dfplayer_RX 16
#define dfplayer_TX 17
#define default_volume 15
#define nss_pin 5
#define rst_pin 14
#define dio0_pin 26
#define dio1_pin 35
#define dio2_pin 34
#define OLED_RESET -1

// LoRa commands
#define STATUS_REQUEST 0x01
#define RESET_REQUEST 0x02
#define DATA_COLLECTION_REQUEST 0x03
#define ALERT_NOTIFICATION 0xFF


// Function prototypes
void playAlertSound();
void handle_command(uint8_t command);
int getLatestDistance();

// Global variables
uint8_t mydata[50] = "Device online";
static osjob_t sendjob;

#ifdef DEV_UNIT
  const unsigned TX_INTERVAL = 30;
#else
  const unsigned TX_INTERVAL = 120;
#endif
bool isFirstTransmission = true;

// LoRa configuration
static const u1_t PROGMEM DEVEUI[8]  = _DEVEUI;
static const u1_t PROGMEM APPEUI[8]  = _APPEUI;
static const u1_t PROGMEM APPKEY[16] = _APPKEY;

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

const lmic_pinmap lmic_pins = {
    .nss = nss_pin,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = rst_pin,
    .dio = {dio0_pin, dio1_pin, dio2_pin},
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
LIDARLite_v3HP myLidarLite;
HardwareSerial myDFPlayerSerial(2);
DFRobotDFPlayerMini myDFPlayer;

TaskHandle_t led_blink_task_handle = NULL;
TaskHandle_t measureDistance_task_handle = NULL;
TaskHandle_t averageCalculation_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t alert_semaphore;
SemaphoreHandle_t distance_semaphore;

int distanceMeasurements[NUM_MEASUREMENTS] = {0};
int measurementIndex = 0;
int baselineDistance = 0;
int thresholdDistance = 0;
unsigned long int measurementNumber = 0;

// Add this helper function for printing hex values
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

// Modify the do_send function to avoid repeated sends
void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // if (isFirstTransmission) {
        //     isFirstTransmission = false;
        //     strcpy((char*)mydata, "Routine status");
        // }
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, strlen((char*)mydata), 0);
        Serial.println(F("Packet queued"));
        Serial.print("Sending message: ");
        Serial.println((char*)mydata);
        Serial.print("Frame counter: ");
        Serial.println(LMIC.seqnoUp);
        // Next TX should reutrn to status
        strcpy((char*)mydata, "Routine status");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// void do_send(osjob_t* j) {
//     // Check if there is not a current TX/RX job running
//     if (LMIC.opmode & OP_TXRXPEND) {
//         Serial.println(F("OP_TXRXPEND, not sending"));
//     } else {
//         // Prepare upstream data transmission at the next possible time.
//         LMIC_setTxData2(1, mydata, strlen((char*)mydata), 0);
//         Serial.println(F("Packet queued"));
//         Serial.print("Sending message: ");
//         Serial.println((char*)mydata);
        
//         // If it's the first transmission, change to "Routine status" after sending
//         if (isFirstTransmission) {
//             isFirstTransmission = false;
//             strcpy((char*)mydata, "Routine status");
//         }
//     }
//     // Schedule next transmission
//     os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
// }

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            // Send "Device online" message
            strcpy((char*)mydata, "Device online");
            do_send(&sendjob);
            break;
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                        Serial.print("0");
                    }
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                if (LMIC.dataLen > 0) {
                    handle_command(LMIC.frame[LMIC.dataBeg]);
                }
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        default:
            Serial.println(F("Unknown event"));
            Serial.println((unsigned) ev);
            break;
    }
}

void handle_command(uint8_t command) {
    switch(command) {
        case STATUS_REQUEST:
            Serial.println("Received STATUS_REQUEST");
            strcpy((char*)mydata, "Status: Device is operational");
            do_send(&sendjob);
            break;
        case RESET_REQUEST:
            Serial.println("Received RESET_REQUEST");
            strcpy((char*)mydata, "Reset: Device is resetting");
            do_send(&sendjob);
            // ESP.restart();
            break;
        case DATA_COLLECTION_REQUEST:
            Serial.println("Received DATA_COLLECTION_REQUEST");
            {
                int distance = getLatestDistance();
                snprintf((char*)mydata, sizeof(mydata), "Data: Latest distance is %d cm", distance);
                do_send(&sendjob);
            }
            break;
        case ALERT_NOTIFICATION:
            #if defined(DEV_UNIT) || defined(WEARABLE_UNIT)
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.println("DEVALERT");
                display.display();
            #endif
            #if defined(DEV_UNIT) || defined(MUSICBOX_UNIT)
                playAlertSound();
            #endif
            Serial.println("Alert notification received");
            break;
        default:
            Serial.println("Unknown command received");
            break;
    }
}

void led_blink_task(void *pvParameters) {
    pinMode(LED, OUTPUT);
    while (true) {
        digitalWrite(LED, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        digitalWrite(LED, LOW);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void lcd_init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.display();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    display.clearDisplay();
    display.display();
}

void measureDistance_task(void *pvParameters) {
    while (true) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        myLidarLite.takeRange();
        int distance = myLidarLite.readDistance();
        xSemaphoreGive(i2c_mutex);

        distanceMeasurements[measurementIndex] = distance;
        measurementIndex = (measurementIndex + 1) % NUM_MEASUREMENTS;
        measurementNumber++;
        xSemaphoreGive(distance_semaphore);

        vTaskDelay(1);
    }
}

void playAlertSound()
{
  myDFPlayer.play(1); // Play the first track on the SD card
}

// Modify the averageCalculation_task to add more debug output and avoid constant alerts
void averageCalculation_task(void *pvParameters) {
    static uint32_t lastAlertTime = 0;
    const uint32_t alertCooldown = 100; // 60 seconds cooldown between alerts

    while (true) {
        if (xSemaphoreTake(distance_semaphore, portMAX_DELAY) == pdTRUE) {
            int sum = 0;
            for (int i = 0; i < NUM_MEASUREMENTS; i++) {
                sum += distanceMeasurements[i];
            }
            int averageDistance = sum / NUM_MEASUREMENTS;

            int distance = distanceMeasurements[(measurementIndex - 1 + NUM_MEASUREMENTS) % NUM_MEASUREMENTS];
            // Serial.printf("Current distance: %d cm, Average: %d cm, Baseline: %d cm, Threshold: %d cm\n", 
            //               distance, averageDistance, baselineDistance, thresholdDistance);

            uint32_t currentTime = millis();
            if (abs(distance - baselineDistance) > thresholdDistance && 
                (currentTime - lastAlertTime > alertCooldown)) {
                strcpy((char*)mydata, "Alert: Distance threshold exceeded");
                do_send(&sendjob);
                lastAlertTime = currentTime;
                Serial.println("Alert triggered!");
                #ifdef DEV_UNIT
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.println("ALERT!");
                display.display();
                playAlertSound();
                #endif
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Check every second instead of continuously
    }
}

void calibrateSensor() {
    int sum = 0;
    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        myLidarLite.takeRange();
        vTaskDelay(10 / portTICK_PERIOD_MS);
        int distance = myLidarLite.readDistance();
        xSemaphoreGive(i2c_mutex);

        sum += distance;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    baselineDistance = sum / NUM_MEASUREMENTS;
    thresholdDistance = baselineDistance * THRESHOLD_PERCENTAGE;
    Serial.printf("Calibration complete. Baseline: %d cm, Threshold: %d cm\n", baselineDistance, thresholdDistance);
}

void lora_task(void *pvParameters) {
    while (true) {
        os_runloop_once();
        vTaskDelay(1);
    }
}

int getLatestDistance() {
    return distanceMeasurements[(measurementIndex - 1 + NUM_MEASUREMENTS) % NUM_MEASUREMENTS];
}

void setup() {
    Serial.begin(115200);

    #if defined(DEV_UNIT) || defined(WEARABLE_UNIT)
    lcd_init();
    #endif

    #if defined(DEV_UNIT) || defined(LIDAR_UNIT)
    myLidarLite.configure(0);
    #endif

    i2c_mutex = xSemaphoreCreateMutex();
    alert_semaphore = xSemaphoreCreateBinary();
    distance_semaphore = xSemaphoreCreateBinary();

    if (i2c_mutex == NULL || alert_semaphore == NULL || distance_semaphore == NULL) {
        Serial.println("Failed to create semaphores");
        while (1);
    }

    #if defined(DEV_UNIT) || defined(LIDAR_UNIT)
    calibrateSensor();
    #endif

    #if defined(DEV_UNIT) || defined(MUSICBOX_UNIT)
    myDFPlayerSerial.begin(9600, SERIAL_8N1, dfplayer_RX, dfplayer_TX);
    if (!myDFPlayer.begin(myDFPlayerSerial)) {
        Serial.println(F("Unable to begin DFPlayer Mini"));
        while (true);
    }
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(default_volume);
    #endif

    os_init();
    LMIC_reset();
    LMIC_startJoining();
    

    #if defined(DEV_UNIT) || defined(WEARABLE_UNIT)
    xTaskCreatePinnedToCore(led_blink_task, "blinkled", 2048, NULL, 1, &led_blink_task_handle, 0);
    #endif

    #if defined(DEV_UNIT) || defined(LIDAR_UNIT)
    xTaskCreatePinnedToCore(measureDistance_task, "getDistance", 2048, NULL, 3, &measureDistance_task_handle, 0);
    xTaskCreatePinnedToCore(averageCalculation_task, "averageCalc", 2048, NULL, 2, &averageCalculation_task_handle, 0);
    #endif

    xTaskCreatePinnedToCore(lora_task, "loraTask", 4096, NULL, 1, &lora_task_handle, 1);
    // do_send(&sendjob);
}

void loop() {
    // Empty loop since everything is handled by FreeRTOS tasks
}

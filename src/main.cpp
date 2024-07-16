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

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define LED 2

#define NUM_MEASUREMENTS 10 // Number of measurements for averaging
#define THRESHOLD_PERCENTAGE 0.05 // Threshold percentage for detecting crossing

#define dfplayer_RX 16
#define dfplayer_TX 17

#define default_volume 15

#define nss_pin 5
#define rst_pin 14
#define dio0_pin 26
#define dio1_pin 35
#define dio2_pin 34

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

void do_send(osjob_t* j);

// LoRa keys and configuration
static const u1_t PROGMEM DEVEUI[8]  = _DEVEUI;
void os_getDevEui(u1_t* buf) {
    memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPEUI[8]  = _APPEUI;
void os_getArtEui(u1_t* buf) {
    memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = _APPKEY;
void os_getDevKey(u1_t* buf) {
    memcpy_P(buf, APPKEY, 16);
}

const lmic_pinmap lmic_pins = {
    .nss = nss_pin,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = rst_pin,
    .dio = {dio0_pin, dio1_pin, dio2_pin},
};

uint8_t mydata[] = "Alert triggered!";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
LIDARLite_v3HP myLidarLite; // Create an instance of the LIDARLite_v3HP

HardwareSerial myDFPlayerSerial(2);
DFRobotDFPlayerMini myDFPlayer; // Create an instance of the DFPlayer Mini

TaskHandle_t led_blink_task_handle = NULL;
TaskHandle_t measureDistance_task_handle = NULL;
TaskHandle_t averageCalculation_task_handle = NULL;
TaskHandle_t alert_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;
TaskHandle_t read_lora_data_task_handle = NULL;

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t alert_semaphore;
SemaphoreHandle_t distance_semaphore;
SemaphoreHandle_t lora_semaphore;

int distanceMeasurements[NUM_MEASUREMENTS] = {0};
int measurementIndex = 0;
int baselineDistance = 0;
int thresholdDistance = 0;
unsigned long int measurementNumber = 0;

void led_blink_task(void *pvParameters)
{
  pinMode(LED, OUTPUT);
  while (true)
  {
    digitalWrite(LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void lcd_init()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  display.invertDisplay(false);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void playAlertSound()
{
  myDFPlayer.play(1); // Play the first track on the SD card
}

void alert_task(void *pvParameters)
{
  while (true)
  {
    if (xSemaphoreTake(alert_semaphore, portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("A L E R T");
      display.display();
      playAlertSound();
      xSemaphoreGive(i2c_mutex);
      
      // Send the alert via LoRa
      do_send(&sendjob);

      vTaskDelay(3000 / portTICK_PERIOD_MS); // Display alert for 3 seconds

      // Clear the display after 3 seconds
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      display.clearDisplay();
      display.display();
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(1); // Yield back to the scheduler
  }
}

void measureDistance_task(void *pvParameters)
{
  while (true)
  {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    myLidarLite.takeRange(); // Initiate measurement
    int distance = myLidarLite.readDistance(); // Get distance in centimeters
    xSemaphoreGive(i2c_mutex);

    // Store the distance in the array
    distanceMeasurements[measurementIndex] = distance;
    measurementIndex = (measurementIndex + 1) % NUM_MEASUREMENTS;
    measurementNumber++;
    xSemaphoreGive(distance_semaphore);

    vTaskDelay(1); // Yield back to the scheduler
  }
}

void averageCalculation_task(void *pvParameters)
{
  while (true)
  {
    if (xSemaphoreTake(distance_semaphore, portMAX_DELAY) == pdTRUE)
    {
      // Calculate the average distance
      int sum = 0;
      for (int i = 0; i < NUM_MEASUREMENTS; i++)
      {
        sum += distanceMeasurements[i];
      }
      int averageDistance = sum / NUM_MEASUREMENTS;

      // Check if the current distance deviates significantly from the baseline
      int distance = distanceMeasurements[(measurementIndex - 1 + NUM_MEASUREMENTS) % NUM_MEASUREMENTS];
      if (abs(distance - baselineDistance) > thresholdDistance)
      {
        xSemaphoreGive(alert_semaphore);
      }
    }
    vTaskDelay(1); // Yield back to the scheduler
  }
}

void calibrateSensor()
{
  int sum = 0;
  for (int i = 0; i < NUM_MEASUREMENTS; i++)
  {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    myLidarLite.takeRange(); // Initiate measurement
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int distance = myLidarLite.readDistance(); // Get distance in centimeters
    xSemaphoreGive(i2c_mutex);

    sum += distance;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  baselineDistance = sum / NUM_MEASUREMENTS;
  thresholdDistance = baselineDistance * THRESHOLD_PERCENTAGE;
  Serial.print("Calibration complete. Baseline distance: ");
  Serial.print(baselineDistance);
  Serial.print(" cm, Threshold distance: ");
  Serial.print(thresholdDistance);
  Serial.println(" cm");
}

void onEvent (ev_t ev) {
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
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");

              LMIC_setSeqnoUp(140);
            }
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
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
            break;
    }
}

void do_send(osjob_t* j) {
  // Prepare upstream data transmission at the next possible time.
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare the data to send
    mydata[0] = 'A'; // Example of sending an alert character
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
}

void lora_task(void *pvParameters)
{
  while (true)
  {
    os_runloop_once();
    vTaskDelay(1); // Minimize delay to ensure continuous processing
  }
}


void read_lora_data_task(void *pvParameters)
{
  while (true)
  {
    // Check if there is any received data
    if (LMIC.dataLen) {
      Serial.print(F("Received data: "));
      for (int i = 0; i < LMIC.dataLen; i++) {
        Serial.print((char)LMIC.frame[LMIC.dataBeg + i]);
      }
      Serial.println();
      
      // Clear the received data length
      LMIC.dataLen = 0;
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 60 seconds
  }
}

void setup()
{
  Serial.begin(115200);
  lcd_init();
  myLidarLite.configure(0); // Initialize the LIDARLite v3HP

  i2c_mutex = xSemaphoreCreateMutex();
  alert_semaphore = xSemaphoreCreateBinary();
  distance_semaphore = xSemaphoreCreateBinary();

  if (i2c_mutex == NULL || alert_semaphore == NULL || distance_semaphore == NULL) {
    Serial.println("Failed to create semaphores");
    while (1);
  }

  // Calibrate the sensor
  calibrateSensor();

  // Initialize DFPlayer Mini
  myDFPlayerSerial.begin(9600, SERIAL_8N1, dfplayer_RX, dfplayer_TX); // RX, TX pins
  if (!myDFPlayer.begin(myDFPlayerSerial))
  {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(default_volume); // Set volume value. From 0 to 30

  // Initialize LoRa
  os_init();
  LMIC_reset();
  LMIC_startJoining();

  // TASK CREATION //
  xTaskCreatePinnedToCore(
      led_blink_task,    // Function that should be called
      "blinkled",        // Name of the task (for debugging)
      2048,              // Stack size (bytes)
      NULL,              // Parameter to pass
      1,                 // Task priority
      &led_blink_task_handle, // Task handle
      0);                // Core to pin the task to

  xTaskCreatePinnedToCore(
      measureDistance_task, // Function that should be called
      "getDistance",    // Name of the task (for debugging)
      2048,             // Stack size (bytes)
      NULL,             // Parameter to pass
      3,                // Task priority
      &measureDistance_task_handle, // Task handle
      0);               // Core to pin the task to

  xTaskCreatePinnedToCore(
      averageCalculation_task, // Function that should be called
      "averageCalc",     // Name of the task (for debugging)
      2048,              // Stack size (bytes)
      NULL,              // Parameter to pass
      2,                 // Task priority
      &averageCalculation_task_handle, // Task handle
      0);                // Core to pin the task to

  xTaskCreatePinnedToCore(
      alert_task,       // Function that should be called
      "alertTask",      // Name of the task (for debugging)
      2048,             // Stack size (bytes)
      NULL,             // Parameter to pass
      2,                // Task priority
      &alert_task_handle, // Task handle
      0);               // Core to pin the task to

  // Start the LoRa task on core 1
  xTaskCreatePinnedToCore(
      lora_task,        // Function that should be called
      "loraTask",       // Name of the task (for debugging)
      4096,             // Stack size (bytes)
      NULL,             // Parameter to pass
      1,                // Task priority
      &lora_task_handle,// Task handle
      1);               // Core to pin the task to

  // Start the task to read LoRa data
  xTaskCreatePinnedToCore(
      read_lora_data_task, // Function that should be called
      "readLoraData",      // Name of the task (for debugging)
      2048,                // Stack size (bytes)
      NULL,                // Parameter to pass
      2,                   // Task priority
      &read_lora_data_task_handle, // Task handle
      1);                  // Core to pin the task to
}

void loop()
{
  // Empty loop since everything is handled by FreeRTOS tasks
}

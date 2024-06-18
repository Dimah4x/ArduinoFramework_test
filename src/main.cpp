#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "LIDARLite_v3HP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define LED 2

#define NUM_MEASUREMENTS 10 // Number of measurements for averaging
#define THRESHOLD_PERCENTAGE 0.05 // Threshold percentage for detecting crossing

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
LIDARLite_v3HP myLidarLite; // Create an instance of the LIDARLite_v3HP

TaskHandle_t led_blink_task_handle = NULL;
TaskHandle_t measureDistance_task_handle = NULL;
TaskHandle_t averageCalculation_task_handle = NULL;
TaskHandle_t alert_task_handle = NULL;

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t alert_semaphore;
SemaphoreHandle_t distance_semaphore;

int distanceMeasurements[NUM_MEASUREMENTS] = {0};
int measurementIndex = 0;
int baselineDistance = 0;
int thresholdDistance = 0;

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
      xSemaphoreGive(i2c_mutex);
      vTaskDelay(3000 / portTICK_PERIOD_MS); // Display alert for 3 seconds

      // Clear the display after 3 seconds
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      display.clearDisplay();
      display.display();
      xSemaphoreGive(i2c_mutex);
    }
  }
}

void measureDistance_task(void *pvParameters)
{
  while (true)
  {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    myLidarLite.takeRange(); // Initiate measurement
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    int distance = myLidarLite.readDistance(); // Get distance in centimeters
    xSemaphoreGive(i2c_mutex);

    // Store the distance in the array
    distanceMeasurements[measurementIndex] = distance;
    measurementIndex = (measurementIndex + 1) % NUM_MEASUREMENTS;

    xSemaphoreGive(distance_semaphore);

    vTaskDelay(1 / portTICK_PERIOD_MS);
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

      Serial.print("Average: ");
      Serial.print(averageDistance);
      Serial.print(" cm, Baseline: ");
      Serial.print(baselineDistance);
      Serial.print(" cm, Threshold: ");
      Serial.print(thresholdDistance);
      Serial.println(" cm");

      // Check if the current distance deviates significantly from the baseline
      int distance = distanceMeasurements[(measurementIndex - 1 + NUM_MEASUREMENTS) % NUM_MEASUREMENTS];
      if (abs(distance - baselineDistance) > thresholdDistance)
      {
        xSemaphoreGive(alert_semaphore);
      }
    }
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

void setup()
{
  Serial.begin(115200);
  lcd_init();
  myLidarLite.configure(0); // Initialize the LIDARLite v3HP

  i2c_mutex = xSemaphoreCreateMutex();
  alert_semaphore = xSemaphoreCreateBinary();
  distance_semaphore = xSemaphoreCreateBinary();

  // Calibrate the sensor
  calibrateSensor();

  // TASK CREATION //
  xTaskCreatePinnedToCore(
      led_blink_task,    // Function that should be called
      "blinkled",        // Name of the task (for debugging)
      2048,              // Stack size (bytes)
      NULL,              // Parameter to pass
      1,                 // Task priority
      &led_blink_task_handle, // Task handle
      1);                // Core to pin the task to

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
}

void loop()
{
}

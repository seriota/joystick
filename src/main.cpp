#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

// Define pins for the first joystick
#define JOYSTICK1_VRX 4
#define JOYSTICK1_VRY 2
#define JOYSTICK1_SW 22

// Define pins for the second joystick
#define JOYSTICK2_VRX 35
#define JOYSTICK2_VRY 34
#define JOYSTICK2_SW 16

// Define pins for the lora esp32 v1
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Define LoRa bandwidth and frequency
#define LORA_BANDWIDTH 915E6 // 915 MHz

// Define joystick low-pass filter gain
#define LOW_PASS_GAIN 10

#define DEBOUNCE_DELAY 2 // debounce delay
#define DEBOUNCE_TIME 50 // debounce time for SW1 and SW2

long final_joystick1_vrx = 0;
long final_joystick1_vry = 0;
long final_joystick2_vrx = 0;
long final_joystick2_vry = 0;

int counter = 0;

String payload = "";

byte crc8(const uint8_t *data, size_t len)
{
  uint8_t crc = 0x00;        // Initial value
  uint8_t polynomial = 0x07; // x^8 + x^2 + x + 1

  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ polynomial;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

TaskHandle_t debounceTaskHandleSW1;
TaskHandle_t debounceTaskHandleSW2;
TaskHandle_t payloadTaskHandle;

volatile int joystick1_sw = HIGH;
volatile int joystick2_sw = HIGH;

void debounceSW1Task(void *parameter)
{
  int lastSteadyState = HIGH;
  int lastFlickerableState = HIGH;
  int currentState;
  unsigned long lastDebounceTime = 0;

  while (true)
  {
    currentState = digitalRead(JOYSTICK1_SW);

    // If the switch changed, due to noise or pressing
    if (currentState != lastFlickerableState)
    {
      // Reset the debounce timer
      lastDebounceTime = millis();
      // Save the last flickerable state
      lastFlickerableState = currentState;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME)
    {
      // If the button state has changed and is stable
      if (lastSteadyState == HIGH && currentState == LOW)
      {
        Serial.println("SW1 is pressed");
        joystick1_sw = LOW;
      }
      else if (lastSteadyState == LOW && currentState == HIGH)
      {
        Serial.println("SW1 is released");
        joystick1_sw = HIGH;
      }
      // Even if the button is held down, keep updating the steady state
      // This ensures we're always tracking the current state
      lastSteadyState = currentState;
    }

    vTaskDelay(5);
  }
}

void debounceSW2Task(void *parameter)
{
  int lastSteadyState = HIGH;
  int lastFlickerableState = HIGH;
  int currentState;
  unsigned long lastDebounceTime = 0;

  while (true)
  {
    currentState = digitalRead(JOYSTICK2_SW);

    // If the switch changed, due to noise or pressing
    if (currentState != lastFlickerableState)
    {
      // Reset the debounce timer
      lastDebounceTime = millis();
      // Save the last flickerable state
      lastFlickerableState = currentState;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME)
    {
      // If the button state has changed and is stable
      if (lastSteadyState == HIGH && currentState == LOW)
      {
        Serial.println("SW2 is pressed");
        joystick2_sw = LOW;
      }
      else if (lastSteadyState == LOW && currentState == HIGH)
      {
        Serial.println("SW2 is released");
        joystick2_sw = HIGH;
      }
      // Even if the button is held down, keep updating the steady state
      lastSteadyState = currentState;
    }

    vTaskDelay(5);
  }
}

void payloadTask(void *parameter)
{
  static char buffer[128];     // Buffer for payload
  static char tempBuffer[128]; // Buffer for temporary storage

  while (true)
  {
    final_joystick1_vrx += analogRead(JOYSTICK1_VRX);
    final_joystick1_vry += analogRead(JOYSTICK1_VRY);
    final_joystick2_vrx += analogRead(JOYSTICK2_VRX);
    final_joystick2_vry += analogRead(JOYSTICK2_VRY);

    if (counter % LOW_PASS_GAIN == 0)
    {
      unsigned int j1_vrx = final_joystick1_vrx / LOW_PASS_GAIN;
      unsigned int j1_vry = final_joystick1_vry / LOW_PASS_GAIN;
      unsigned int j2_vrx = final_joystick2_vrx / LOW_PASS_GAIN;
      unsigned int j2_vry = final_joystick2_vry / LOW_PASS_GAIN;

      snprintf(tempBuffer, sizeof(tempBuffer), "%u:%u:%d:%u:%u:%d", j1_vrx, j1_vry, joystick1_sw, j2_vrx, j2_vry, joystick2_sw);
      byte checksum = crc8((const uint8_t *)tempBuffer, strlen(tempBuffer));
      snprintf(buffer, sizeof(buffer), "?%u:%s", checksum, tempBuffer);

      LoRa.beginPacket();
      LoRa.print(buffer);
      LoRa.endPacket();

      Serial.write(buffer);
      Serial.write("\n");

      // Reset
      final_joystick1_vrx = 0;
      final_joystick1_vry = 0;
      final_joystick2_vrx = 0;
      final_joystick2_vry = 0;
      counter = 0;
      buffer[0] = '\0';
    }

    counter++;

    // Feed the watchdog timer to prevent resets
    esp_task_wdt_reset();

    vTaskDelay(5);
  }
}

void setup()
{
  // ! Initialize serial communication
  Serial.begin(115200); // Increase baud rate for faster communication
  Serial.println("Initializing...");

  // Initialize the watchdog timer
  esp_task_wdt_init(30, false); // 30 second timeout, no panic on timeout

  WiFi.mode(WIFI_OFF); // Disable WiFi to save power
  btStop();            // Disable Bluetooth to save power
  delay(1000);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // ! Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(LORA_BANDWIDTH))
  {
    Serial.println(".");
    delay(500);
  }

  // ! Configure joystick pins
  pinMode(JOYSTICK1_SW, INPUT_PULLUP);
  pinMode(JOYSTICK2_SW, INPUT_PULLUP);
  pinMode(JOYSTICK1_VRX, INPUT);
  pinMode(JOYSTICK1_VRY, INPUT);
  pinMode(JOYSTICK2_VRX, INPUT);
  pinMode(JOYSTICK2_VRY, INPUT);

  analogReadResolution(12); // Set the ADC resolution to 12 bits (0-4095)

  // ! Print initialization message
  Serial.println("Joysticks initialized");
  xTaskCreate(debounceSW1Task, "Debounce Task", 1000, NULL, 1, &debounceTaskHandleSW1);
  xTaskCreate(debounceSW2Task, "Debounce Task 2", 1000, NULL, 1, &debounceTaskHandleSW2);
  xTaskCreate(payloadTask, "Payload Task", 10000, NULL, 2, &payloadTaskHandle);

  // ! Register payload task with watchdog
  esp_task_wdt_add(payloadTaskHandle);
}

void loop()
{
  vTaskDelay(100);
}

# Building an Energy-Efficient Wireless Sensor Network Using ESP32, ESP-NOW, and MQTT

In this project, we will create a robust and energy-efficient **wireless sensor network** using **ESP32** boards. Sensor data will be collected from **DHT11 temperature and humidity sensors** and transmitted using **ESP-NOW**, a low-power communication protocol. The data will then be forwarded to an MQTT broker via a bridge ESP32 connected over I2C, solving the challenge of Wi-Fi interruptions while using ESP-NOW.

This project is ideal for scenarios like remote environmental monitoring or smart agriculture, where efficient data transmission and power conservation are critical.

---

## Features
1. **Energy Efficiency:** Sender nodes use light sleep mode to conserve power.
2. **Robust Wireless Communication:** ESP-NOW ensures low-latency, Wi-Fi-independent communication.
3. **Scalable Data Management:** Data is forwarded to an MQTT broker for remote monitoring and integration with IoT platforms.
4. **Battery Life Optimization:** Includes calculations to estimate battery runtime for sender nodes powered by a 3.7V 1500mAh battery.

---

## System Architecture

### System Diagram
![Architecture Diagram](https://via.placeholder.com/1024x768.png?text=Architecture+Diagram)

---

### Components
1. **Sender Nodes (2 ESP32s):**
   - Read data from DHT11 sensors.
   - Transmit data to the receiver via ESP-NOW.
   - Enter light sleep mode between transmissions for energy efficiency.

2. **Receiver Node (1 ESP32):**
   - Collects data from both sender nodes using ESP-NOW.
   - Forwards the data to the bridge node over I2C communication.

3. **Bridge Node (1 ESP32):**
   - Receives data from the receiver node via I2C.
   - Publishes the data to an MQTT broker for IoT integration.

---

## Step 1: Hardware Setup

### Materials Required
- 4x **ESP32 boards**
- 2x **DHT11 Sensors**
- Jumper Wires
- Breadboard
- Power supply or batteries for sender nodes

### Wiring Diagrams

#### Sender Node:
| Component | ESP32 Pin |
|-----------|-----------|
| VCC       | 3.3V      |
| GND       | GND       |
| Data      | GPIO4     |

#### Receiver Node and Bridge Node:
- Receiver and Bridge are connected via I2C:
  - **SDA (Receiver):** GPIO21 -> **SDA (Bridge):** GPIO21
  - **SCL (Receiver):** GPIO22 -> **SCL (Bridge):** GPIO22

---

## Step 2: Configuring Sender Nodes

The sender nodes will:
1. Read temperature and humidity from the DHT11 sensor.
2. Send the data via ESP-NOW to the receiver ESP32.
3. Enter light sleep mode to save energy.

### Sender Node Code

```cpp
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Structure to hold data
typedef struct struct_message {
  float temperature;
  float humidity;
} struct_message;

struct_message sensorData;

uint8_t receiverAddress[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // Replace with receiver MAC

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  dht.begin();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Read sensor data
  sensorData.temperature = dht.readTemperature();
  sensorData.humidity = dht.readHumidity();

  if (isnan(sensorData.temperature) || isnan(sensorData.humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Send data via ESP-NOW
  esp_now_send(receiverAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  Serial.println("Data sent!");

  // Enter light sleep mode
  esp_sleep_enable_timer_wakeup(10 * 1000000); // 10 seconds
  esp_light_sleep_start();
}
```
## Step 3: Configuring the Receiver Node
# The receiver node will:

1. Receive data from both sender nodes via ESP-NOW.
2. Forward the data to the bridge node over I2C.
3. Receiver Node Code

```cpp
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// Structure to hold received data
typedef struct struct_message {
  float temperature;
  float humidity;
} struct_message;

struct_message receivedData;

// Callback when data is received
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Wire.beginTransmission(8); // I2C address of bridge node
  Wire.write((uint8_t *)&receivedData, sizeof(receivedData));
  Wire.endTransmission();
  Serial.printf("Data received: Temp: %.2f, Humidity: %.2f\n", receivedData.temperature, receivedData.humidity);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Wire.begin(21, 22); // I2C pins SDA=21, SCL=22

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  delay(1000); // Keep running
}
```
## Step 4: Setting Up the MQTT Bridge

### The bridge node will:

1. Receive sensor data from the receiver node via I2C.
2. Publish the data to the MQTT broker.

### Bridge Node Code

```cpp
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "YourSSID";
const char *password = "YourPassword";
const char *mqtt_server = "YourMQTTBrokerIP";

WiFiClient espClient;
PubSubClient client(espClient);

typedef struct struct_message {
  float temperature;
  float humidity;
} struct_message;

struct_message receivedData;

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  Wire.begin(21, 22); // I2C pins SDA=21, SCL=22
}

void loop() {
  Wire.requestFrom(8, sizeof(receivedData)); // I2C address of receiver node
  if (Wire.available()) {
    Wire.readBytes((char *)&receivedData, sizeof(receivedData));
    char tempStr[8], humStr[8];
    dtostrf(receivedData.temperature, 1, 2, tempStr);
    dtostrf(receivedData.humidity, 1, 2, humStr);

    if (!client.connected()) {
      client.connect("ESP32Bridge");
    }

    client.publish("sensors/temperature", tempStr);
    client.publish("sensors/humidity", humStr);
    Serial.printf("Published: Temp: %s, Humidity: %s\n", tempStr, humStr);
  }

  client.loop();
  delay(2000);
}
```
# Duty Cycle Calculation

The steps to calculate the **Duty Cycle** based on the data provided from the sender and receiver logs.

---

## 1. Definitions

- **Active Time per Cycle (`T_active`)**: 
  - The time the sender is actively transmitting data.
  - From the logs, this is approximately **17 ms** per transmission.

- **Cycle Period (`T_cycle`)**: 
  - The time interval between consecutive transmissions.

## 2. Data Analysis

From the sender timestamps:
- `5194 - 168 = 5026 ms`
- `10219 - 5194 = 5025 ms`
- `15244 - 10219 = 5025 ms`
- `20269 - 15244 = 5025 ms`
- `25294 - 20269 = 5025 ms`

### Average Cycle Period:

T_cycle ≈ 5025 ms = 5.025 seconds

## 3. Formula

The Duty Cycle is calculated using the formula:
  Duty Cycle = (T_active / T_cycle) × 100

---

## 4. Calculation

### Step 1: Calculate Active Time (`T_active`):
From the receiver logs, the time taken per transmission is:
  T_active = 17 ms = 0.017 seconds

### Step 2: Calculate Duty Cycle:
Substitute the values into the formula:
  Duty Cycle = (0.017 / 5.025) × 100

Perform the division:
  0.017 / 5.025 ≈ 0.003384

Multiply by 100:
  Duty Cycle ≈ 0.34%
---

## Final Answer:

The duty cycle for this sender is approximately:
0.34%

### Battery Life Estimation

Using a 3.7V 1500mAh Li-ion battery:

Battery Life (hours) = Battery Capacity (mAh) / I_avg ≈ 1500 / 3.55 ≈ 422.5 hours

In days:

Battery Life (days) ≈ 422.5 / 24 ≈ 17.6 days

## Conclusion

This project demonstrates how to build a low-power wireless sensor network with ESP32, ESP-NOW, and MQTT. With a duty cycle of approximately 1.48% and a battery life of over 17 days, this setup is ideal for scenarios requiring periodic data transmission and efficient energy management.


#include <Firebase_ESP_Client.h>
#include <DHT.h>
#include <WiFi.h>

// Define the pin numbers
#define DHTPIN 4        // Pin to connect DHT sensor (GPIO number)
#define DHTTYPE DHT11   // Define DHT 11 sensor type
#define LED_LIGHT 15    // Pin for blue LED controlled by LDR
#define LDR_PIN 13      // Pin for LDR sensor
#define MQ7_PIN 34      // Pin for MQ7 gas sensor
#define BUZZER_PIN 2    // Pin for buzzer
#define LED_GAS 12      // Pin for red LED controlled by MQ7
#define LED_GREEN 14    // Pin for green LED

DHT dht(DHTPIN, DHTTYPE);

// Replace with your own Firebase project credentials
#define FIREBASE_HOST "https://fosstesting-d60bf-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyDP26rjuoJR7ojt5LTK-fGciR1LQhq-HU4"
#define WIFI_SSID "kunal"
#define WIFI_PASSWORD "kunal2004"

// Define Firebase objects
FirebaseData firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

unsigned long previousMillisLDR = 0;
unsigned long previousMillisGas = 0;
unsigned long previousMillisDHT = 0;
unsigned long previousMillisBuzzer = 0;

const long intervalLDR = 500;    // Interval at which to blink LDR LED (in milliseconds)
const long intervalGas = 1000;   // Interval at which to check gas sensor (in milliseconds)
const long intervalDHT = 2000;   // Interval at which to read DHT sensor (in milliseconds)
const long intervalBuzzer = 100; // Interval at which to toggle the buzzer and gas LED (in milliseconds)

bool isGasDetected = false;
bool buzzerState = false;
bool ldrLedState = false;

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(LED_LIGHT, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  pinMode(MQ7_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GAS, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Assign Firebase credentials
  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  // Initialize Firebase
  Firebase.begin(&firebaseConfig, &firebaseAuth);
}

bool updateFirebase(const String &path, const String &value) {
  int retryCount = 0;
  const int maxRetries = 3;
  
  while (retryCount < maxRetries) {
    if (Firebase.RTDB.setString(&firebaseData, path, value)) {
      return true;
    } else {
      Serial.println("Failed to update " + path);
      Serial.println(firebaseData.errorReason());
      retryCount++;
      delay(1000); // Wait a bit before retrying
    }
  }
  return false;
}

void readDHTSensor() {
  float tempC = dht.readTemperature(false);  // Read temperature in Celsius
  float humidity = dht.readHumidity();       // Read humidity

  Serial.print("Temp: ");
  Serial.print(tempC);
  Serial.print(" C, Hum: ");
  Serial.print(humidity);
  Serial.println("%");

  // Update Firebase immediately with the latest temperature and humidity
  if (updateFirebase("/Temperature", String(tempC))) {
    Serial.println("Temperature updated successfully");
  } else {
    Serial.println("Failed to update temperature");
  }

  if (updateFirebase("/Humidity", String(humidity))) {
    Serial.println("Humidity updated successfully");
  } else {
    Serial.println("Failed to update humidity");
  }
}

void checkLDR() {
  int lightState = digitalRead(LDR_PIN);
  bool isLdrDark = lightState == HIGH;

  if (isLdrDark) {
    ldrLedState = !ldrLedState;
    digitalWrite(LED_LIGHT, ldrLedState);
  } else {
    digitalWrite(LED_LIGHT, LOW);
  }

  // Update Firebase immediately with the latest light status
  if (updateFirebase("/LightStatus", isLdrDark ? "SUSSY BAKA BEHAVIOUR" : "PEACE")) {
    Serial.println(isLdrDark ? "Light status updated: SUSSY BAKA BEHAVIOUR" : "Light status updated: Roger - PEACE");
  } else {
    Serial.println("Failed to update Light status");
  }
}

void checkGasSensor() {
  int gasState = digitalRead(MQ7_PIN);
  isGasDetected = gasState == LOW;  // Assuming LOW means gas detected

  // Update Firebase immediately with the latest gas status
  if (updateFirebase("/GasStatus", isGasDetected ? "ITS BURNING!" : "PEACE")) {
    Serial.println(isGasDetected ? "Gas status updated: ITS BURNING!" : "Gas status updated: Roger - PEACE");
  } else {
    Serial.println("Failed to update Gas status");
  }
}

void controlBuzzerAndLED() {
  if (isGasDetected) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisBuzzer >= intervalBuzzer) {
      previousMillisBuzzer = currentMillis;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState);
      digitalWrite(LED_GAS, buzzerState);
    }
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_GAS, LOW);
  }
}

void controlGreenLED() {
  bool isLdrLight = digitalRead(LDR_PIN) == LOW;
  digitalWrite(LED_GREEN, isLdrLight && !isGasDetected);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisDHT >= intervalDHT) {
    previousMillisDHT = currentMillis;
    readDHTSensor();
  }

  if (currentMillis - previousMillisLDR >= intervalLDR) {
    previousMillisLDR = currentMillis;
    checkLDR();
  }

  if (currentMillis - previousMillisGas >= intervalGas) {
    previousMillisGas = currentMillis;
    checkGasSensor();
  }

  // Always control buzzer and LED independently
  controlBuzzerAndLED();
  controlGreenLED();
}

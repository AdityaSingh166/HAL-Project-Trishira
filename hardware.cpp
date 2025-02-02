#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_PCF8574.h>
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Sensor instances
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;
LiquidCrystal_PCF8574 lcd(0x27);

// GPS instances
#define RX_PIN D7
#define TX_PIN D8
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

// Threshold values for fall detection
#define FREE_FALL_THRESHOLD 0.5
#define IMPACT_THRESHOLD 20.0
#define GYRO_THRESHOLD 100.0

// SpO2 & Heart Rate Variables
#define DATA_BUFFER_SIZE 100
uint32_t irBuffer[DATA_BUFFER_SIZE];
uint32_t redBuffer[DATA_BUFFER_SIZE];
int spo2, heartRate;
int8_t validSPO2, validHeartRate;

// Wi-Fi Credentials
#define WIFI_SSID "GATCSE1-2.4Ghz"  // ⚠️ REPLACE WITH YOUR WIFI SSID
#define WIFI_PASSWORD "global123" // ⚠️ REPLACE WITH YOUR WIFI PASSWORD

// Firebase Credentials
#define FIREBASE_HOST "https://thyroid-detection-6d511-default-rtdb.asia-southeast1.firebasedatabase.app/" // ⚠️ REPLACE WITH YOUR FIREBASE HOST
#define FIREBASE_AUTH "0FZyQnxYtngCHz2CHwLGxikGDF9BxLZAdka42Ht0" // ⚠️ REPLACE WITH YOUR FIREBASE AUTH TOKEN

FirebaseConfig config;
FirebaseAuth auth;
FirebaseData firebaseData;

#define FASTAPI_SERVER "http://172.16.10.211:8000/predict" // ⚠️ REPLACE WITH YOUR FASTAPI SERVER IP

// Button pins for user response
#define YES_BUTTON D3
#define NO_BUTTON D4

WiFiClient client;
HTTPClient http;

bool displayingHealthData = false;

void setup() {
    Serial.begin(115200);
    Wire.begin(D2, D1);

    // Connect to Wi-Fi
    connectWiFi();

    // Configure Firebase
    Serial.println("🔗 Connecting to Firebase...");
    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    if (Firebase.ready()) Serial.println("✅ Firebase Connected!");
    else Serial.println("❌ Firebase Connection Failed!");

    // Initialize MLX90614
    delay(1000);
    if (!mlx.begin()) {
        Serial.println("❌ MLX90614 Not Found!");
        while (1);
    } else {
        Serial.println("✅ MLX90614 Initialized!");
    }

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("❌ MPU6050 Not Found!");
        while (1);
    } else {
        Serial.println("✅ MPU6050 Initialized!");
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 not found!");
        lcd.setCursor(0, 1);
        lcd.print("Sensor Error!");
        while (1);
    }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);

    // Initialize LCD
    lcd.begin(16, 2);
    lcd.setBacklight(255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    delay(2000);
    lcd.clear();

    // Initialize buttons
    pinMode(YES_BUTTON, INPUT_PULLUP);
    pinMode(NO_BUTTON, INPUT_PULLUP);

    // Initialize GPS
    gpsSerial.begin(9600);
    Serial.println("📡 Waiting for GPS Fix...");
}

void connectWiFi() {
    Serial.print("🔗 Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 15) {
        delay(1000);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) Serial.println("\n✅ Connected to Wi-Fi!");
    else Serial.println("\n❌ Wi-Fi Connection Failed!");
}

void loop() {
    float ambientTemp = mlx.readAmbientTempC();
    float objectTemp = mlx.readObjectTempC();

    if (isnan(ambientTemp) || ambientTemp > 1000 || ambientTemp < -50) {
        Serial.println("⚠️ Invalid Ambient Temp! Retrying...");
        delay(100);
        ambientTemp = mlx.readAmbientTempC();
    }
    if (isnan(objectTemp) || objectTemp > 1000 || objectTemp < -50) {
        Serial.println("⚠️ Invalid Object Temp! Retrying...");
        delay(100);
        objectTemp = mlx.readObjectTempC();
    }

    // Send temperature data to Firebase and FastAPI
    sendTemperatureData(ambientTemp, objectTemp);

    // Read accelerometer data for impact detection
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float accelMagnitude = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));

    // Check for impact and handle alerts
    handleImpactDetection(accelMagnitude);

    // Check finger presence and display health data or temperature
    handleSensorDisplay();

    // GPS data processing
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    delay(500);
}

void sendTemperatureData(float ambientTemp, float objectTemp) {
    if (!isnan(ambientTemp) && ambientTemp < 1000) {
        Firebase.setFloat(firebaseData, "/sensor/ambient_temperature", ambientTemp);
        Serial.println("✅ Ambient Temperature Sent to Firebase.");
    }
    if (!isnan(objectTemp) && objectTemp < 1000) {
        Firebase.setFloat(firebaseData, "/sensor/temperature", objectTemp);
        Serial.println("✅ Object Temperature Sent to Firebase.");
    }
    sendToFastAPI(ambientTemp, objectTemp);
}

void handleImpactDetection(float accelMagnitude) {
    if (accelMagnitude > IMPACT_THRESHOLD) {
        if (Firebase.setString(firebaseData, "/alerts/impact", "Impact Detected")) { // ✅ Set "Impact Detected"
            Serial.println("\n💥 Impact Detected!");
            Serial.println("✅ Firebase Updated to: Impact Detected under /alerts/impact.");
        } else {
            Serial.println("❌ Failed to update Firebase for impact alert.");
        }

        // Get GPS data
        String gpsData[2];
        if (getGPSLocation(gpsData)) {
            if (Firebase.setString(firebaseData, "/alerts/impact_latitude", gpsData[0])) { // ✅ Send latitude
                Serial.println("✅ Latitude Sent to Firebase under /alerts/impact_latitude.");
            } else {
                Serial.println("❌ Failed to update Firebase for impact latitude.");
            }
            if (Firebase.setString(firebaseData, "/alerts/impact_longitude", gpsData[1])) { // ✅ Send longitude
                Serial.println("✅ Longitude Sent to Firebase under /alerts/impact_longitude.");
            } else {
                Serial.println("❌ Failed to update Firebase for impact longitude.");
            }
        } else {
            Serial.println("❌ GPS data not found after impact.");
        }

        // LCD display for impact and emergency prompt
        displayImpactDetectedPrompt();
        promptEmergency();
        displayingHealthData = false;

    } else {
        if (Firebase.setString(firebaseData, "/alerts/impact", "No Impact")) { // ✅ Set "No Impact"
            Serial.println("✅ Firebase Updated to: No Impact under /alerts/impact.");
        } else {
            Serial.println("❌ Failed to update Firebase for no impact status.");
        }
    }
}

void displayImpactDetectedPrompt() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Impact Detected!");
    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Call Emergency?");
    lcd.setCursor(0, 1);
    lcd.print("Press Y/N");
}


void handleSensorDisplay() {
    if (particleSensor.check() && particleSensor.getRed() > 5000) {
        displayHealthData();
        displayingHealthData = true;
    } else {
        if (displayingHealthData) { // Clear LCD only when transitioning from health data to temp
            lcd.clear(); // Clear LCD when finger is removed after health data display
            displayingHealthData = false;
        }
        if (!displayingHealthData) {
            displayTemperature(mlx.readAmbientTempC(), mlx.readObjectTempC()); // Refresh temperature display
        }
    }
}


void displayTemperature(float ambientTemp, float objectTemp) {
    if (!displayingHealthData) { // Only update temperature if not displaying health data
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Amb: "); lcd.print(ambientTemp, 1); lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Obj: "); lcd.print(objectTemp, 1); lcd.print("C");
        Serial.print("Amb: "); Serial.print(ambientTemp); Serial.print(" | Obj: "); Serial.println(objectTemp);
    }
}


void displayHealthData() {
    collectSpO2Data();
    calculateSpO2HeartRate();
    float glucose = predictGlucose(heartRate, spo2);
    float cholesterol = predictCholesterol(glucose);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HR:"); lcd.print(heartRate); lcd.print("bpm");
    lcd.setCursor(9, 0);
    lcd.print(" O2:"); lcd.print(spo2); lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("G:"); lcd.print(glucose); lcd.print("mg/dL");
    lcd.setCursor(8, 1);
    lcd.print("C:"); lcd.print(cholesterol); lcd.print("mg/dL");

    Serial.print("HR: "); Serial.print(heartRate);
    Serial.print(" | O2: "); Serial.print(spo2);
    Serial.print(" | Glucose: "); Serial.print(glucose);
    Serial.print(" | Cholesterol: "); Serial.println(cholesterol);

    sendHealthDataToFirebase(spo2, heartRate, glucose, cholesterol);
}

void collectSpO2Data() {
    for (int i = 0; i < DATA_BUFFER_SIZE; i++) {
        while (!particleSensor.check());
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
    }
}

void calculateSpO2HeartRate() {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, DATA_BUFFER_SIZE, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void sendHealthDataToFirebase(int spo2, int heartRate, float glucose, float cholesterol) {
    Firebase.setInt(firebaseData, "/sensor/spo2", spo2);
    Firebase.setInt(firebaseData, "/sensor/heart_rate", heartRate);
    Firebase.setFloat(firebaseData, "/sensor/glucose", glucose);
    Firebase.setFloat(firebaseData, "/sensor/cholesterol", cholesterol);
    Serial.println("✅ Health Data Sent to Firebase.");
}


float predictGlucose(float bpm, float spo2) {
    return (0.5 * bpm + 1.2 * spo2);
}

float predictCholesterol(float glucose) {
    return glucose * 1.2;
}

void promptEmergency() {
    unsigned long startTime = millis();
    bool responded = false;

    while (millis() - startTime < 10000) {
        if (digitalRead(YES_BUTTON) == LOW) { handleEmergencyCall(); responded = true; break; }
        if (digitalRead(NO_BUTTON) == LOW) { handleCallCancelled(); responded = true; break; }

        if (Serial.available()) {
            String response = Serial.readStringUntil('\n');
            response.trim();
            if (response.equalsIgnoreCase("YES")) { handleEmergencyCall(); responded = true; break; }
            if (response.equalsIgnoreCase("NO")) { handleCallCancelled(); responded = true; break; }
        }
        delay(100);
    }
    if (!responded) callEmergencyServices();
}

void handleEmergencyCall() {
    Serial.println("🚨 Calling Emergency Services...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calling Help...");
    delay(2000);
    callEmergencyServices();
}

void handleCallCancelled() {
    Serial.println("User Cancelled Emergency Call");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Call Cancelled");
    delay(2000);
}

void callEmergencyServices() {
    Serial.println("📞 Dialing 112...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dialing 112...");
    delay(2000);
    displayGPSLocation(); // ⭐️ Moved displayGPSLocation() here, after "Dialing 112..."
    updateFirebaseEmergencyStatus("Emergency Dialing"); // ✅ Update Firebase status

    displayingHealthData = false;
}

void updateFirebaseEmergencyStatus(String status) {
    if (Firebase.setString(firebaseData, "/alerts/emergency", status)) { // Use status parameter
        Serial.print("✅ Firebase Updated to: "); Serial.print(status); Serial.println(" under /alerts/emergency.");
    } else {
        Serial.println("❌ Failed to update Firebase for emergency status.");
    }
}


void sendToFastAPI(float ambient, float object) {
    if (WiFi.status() == WL_CONNECTED) {
        http.begin(client, FASTAPI_SERVER);
        http.addHeader("Content-Type", "application/json");

        String postData = "{ \"ambient_temp\": " + String(ambient) + ", \"object_temp\": " + String(object) + " }";
        Serial.print("➡️ Sending to FastAPI: ");
        Serial.println(postData);

        int httpResponseCode = http.POST(postData);
        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("📩 Response from FastAPI:");
            Serial.println(response);
        } else {
            Serial.print("❌ HTTP POST Error Code: ");
            Serial.println(httpResponseCode);
            Serial.println("❌ Error Sending Data to FastAPI!");
        }
        http.end();
    } else {
        Serial.println("❌ No Wi-Fi Connection. Cannot send data.");
    }
}

bool getGPSLocation(String gpsData[2]) {
    unsigned long gpsTimeout = millis() + 10000;
    String latitude = "No GPS";
    String longitude = "No GPS";

    while (millis() < gpsTimeout) {
        while (gpsSerial.available()) {
            gps.encode(gpsSerial.read());
        }

        if (gps.location.isUpdated()) {
            latitude = String(gps.location.lat(), 6);
            longitude = String(gps.location.lng(), 6);

            //displayGpsOnLcd(latitude, longitude); // Call LCD display function

            Serial.print("Latitude: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);

            gpsData[0] = latitude;
            gpsData[1] = longitude;
            return true;
        }
        delay(100);
    }

    Serial.println("❌ GPS Timeout: No data received");
    displayGpsTimeoutOnLcd(); // Call GPS timeout display function
    gpsData[0] = "No GPS";
    gpsData[1] = "No GPS";
    return false;
}

void displayGpsOnLcd(String latitude, String longitude) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lat: ");
    lcd.print(latitude);
    lcd.setCursor(0, 1);
    lcd.print("Lon: ");
    lcd.print(longitude);
    delay(5000);
}

void displayGpsTimeoutOnLcd() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GPS Timeout");
    lcd.setCursor(0, 1);
    lcd.print("No GPS Data");
    delay(2000);
}


void displayGPSLocation() {
    unsigned long gpsTimeout = millis() + 10000;

    while (millis() < gpsTimeout) {
        while (gpsSerial.available()) {
            gps.encode(gpsSerial.read());
        }

        if (gps.location.isUpdated()) {
            displayGpsOnLcd(String(gps.location.lat(), 6), String(gps.location.lng(), 6));
            delay(5000);

            Serial.print("Latitude: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            delay(5000);

            Firebase.setString(firebaseData, "/alerts/emergency_lat", String(gps.location.lat(), 6));
            Firebase.setString(firebaseData, "/alerts/emergency_lon", String(gps.location.lng(), 6));
            Serial.println("✅ GPS data sent to Firebase during emergency call.");
            return;
        }
        delay(100);
    }
    displayGpsTimeoutOnLcd();
}
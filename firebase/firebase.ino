#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// =========================================================================
// 1. NETWORK SETTINGS
// =========================================================================
const char* ssid = "Chief";          
const char* password = "87655678";  

// =========================================================================
// 2. FIREBASE CLOUD SETTINGS
// =========================================================================
const String databaseURL = "https://motorseveritymonitor-default-rtdb.asia-southeast1.firebasedatabase.app/patientData.json";

// =========================================================================
// 3. SENSOR SETTINGS
// =========================================================================
const int MPU_ADDR = 0x68; 
int16_t ax, ay, az, gx, gy, gz;

String dataBatch = "";
int readingCount = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // USB stability delay

  // SuperMini specific I2C pins
  Wire.begin(2, 3); 

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);

  // Connect to the Internet
  Serial.print("\nConnecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
}

void loop() {
  // Read 6-Axis Movement Data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 
  
  // STRICT SAFETY CHECK: Only process data if the sensor actually answers
  if (Wire.available() == 14) {
    ax = Wire.read()<<8 | Wire.read(); 
    ay = Wire.read()<<8 | Wire.read(); 
    az = Wire.read()<<8 | Wire.read(); 
    Wire.read()<<8 | Wire.read(); // Skip temperature
    gx = Wire.read()<<8 | Wire.read(); 
    gy = Wire.read()<<8 | Wire.read(); 
    gz = Wire.read()<<8 | Wire.read(); 

    // Format the numbers into a CSV string separated by semicolons
    dataBatch += String(ax) + "," + String(ay) + "," + String(az) + "," + 
                 String(gx) + "," + String(gy) + "," + String(gz) + ";";
    
    readingCount++;

    // Once the bucket is full (50 readings = ~1 second), blast it to Firebase
    if (readingCount >= 50) {
      
      // =================================================================
      // Print the data batch to the Serial Monitor for debugging
      // =================================================================
      Serial.println("\n>>> Preparing to send 1 second of data...");
      Serial.println("Payload: " + dataBatch);
      
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(databaseURL);
        http.addHeader("Content-Type", "application/json");

        String jsonPayload = "{\"data\":\"" + dataBatch + "\"}";
        int responseCode = http.POST(jsonPayload);

        if (responseCode > 0) {
          Serial.println("Batch Uploaded! HTTP Code: " + String(responseCode));
        } else {
          Serial.println("Upload Failed. Error: " + String(responseCode));
        }
        http.end();
      }
      
      // Empty the bucket to start collecting the next second of data
      dataBatch = "";
      readingCount = 0;
    }
  } else {
    // If the sensor fails, warn the user and attempt a restart
    Serial.println("WARNING: Sensor Disconnected! Got -1 values. Check wiring/power.");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);
    Wire.endTransmission(true);
  }

  // 20-millisecond delay ensures a perfect 50Hz sample rate for the ML analysis
  delay(20); 
}
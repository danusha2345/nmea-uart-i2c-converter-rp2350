/*
 * NMEA Converter with Clear GPIO Names
 * For Waveshare RP2350-Zero
 * 
 * SELF TEST: Connect GPIO0 to GPIO1 with a wire
 * NORMAL USE: Connect GPS TX to GPIO1
 */

#include <Arduino.h>
#include <Wire.h>

// ====== EXPLICIT GPIO PIN DEFINITIONS ======
#define GPIO0  0   // Physical pin 0 on right side - UART TX
#define GPIO1  1   // Physical pin 1 on right side - UART RX  
#define GPIO4  4   // I2C SDA
#define GPIO5  5   // I2C SCL

// Use explicit GPIO names for clarity
#define UART_TX_GPIO  GPIO0
#define UART_RX_GPIO  GPIO1
#define I2C_SDA_GPIO  GPIO4
#define I2C_SCL_GPIO  GPIO5

// Protocol Configuration
#define I2C_SLAVE_ADDRESS 0x42
#define UART_BAUD_RATE 115200
#define NMEA_BUFFER_SIZE 4096

// Self test mode
#define ENABLE_SELF_TEST true

// Simple buffer
char nmeaBuffer[NMEA_BUFFER_SIZE];
int bufferPos = 0;
int sentenceCount = 0;
unsigned long lastTestTime = 0;

void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  
  Serial.println("\n=====================================");
  Serial.println("  NMEA Converter - Clear GPIO Test  ");
  Serial.println("    For Waveshare RP2350-Zero      ");
  Serial.println("=====================================");
  
  // Initialize UART with explicit GPIO names
  Serial.print("\nInitializing UART on Serial1:");
  Serial.print("\n  TX: GPIO"); Serial.print(UART_TX_GPIO);
  Serial.print("\n  RX: GPIO"); Serial.println(UART_RX_GPIO);
  
  Serial1.setTX(UART_TX_GPIO);  // Set GPIO0 as TX
  Serial1.setRX(UART_RX_GPIO);  // Set GPIO1 as RX
  Serial1.begin(UART_BAUD_RATE);
  
  // Initialize I2C
  Serial.print("\nInitializing I2C:");
  Serial.print("\n  SDA: GPIO"); Serial.print(I2C_SDA_GPIO);
  Serial.print("\n  SCL: GPIO"); Serial.println(I2C_SCL_GPIO);
  
  Wire.setSDA(I2C_SDA_GPIO);
  Wire.setSCL(I2C_SCL_GPIO);
  Wire.begin(I2C_SLAVE_ADDRESS);
  
  if (ENABLE_SELF_TEST) {
    Serial.println("\n[SELF TEST MODE]");
    Serial.println("Connect GPIO0 to GPIO1 for loopback test");
    Serial.println("Starting in 2 seconds...\n");
    delay(2000);
  } else {
    Serial.println("\n[READY] Waiting for NMEA data on GPIO1...\n");
  }
}

void sendTestNMEA() {
  if (!ENABLE_SELF_TEST) return;
  
  if (millis() - lastTestTime > 2000) {
    lastTestTime = millis();
    
    // Test sentences
    const char* testSentences[] = {
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
      "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
    };
    
    static int testIndex = 0;
    const char* sentence = testSentences[testIndex % 2];
    
    // Send via UART TX (GPIO0)
    Serial.print("[TEST] Sending on GPIO");
    Serial.print(UART_TX_GPIO);
    Serial.print(": ");
    Serial.println(sentence);
    
    Serial1.println(sentence);
    testIndex++;
  }
}

void processIncomingData() {
  // Read from UART RX (GPIO1)
  while (Serial1.available()) {
    char c = Serial1.read();
    
    // Simple buffer logic
    if (c == '$') {
      bufferPos = 0;
    }
    
    if (bufferPos < NMEA_BUFFER_SIZE - 1) {
      nmeaBuffer[bufferPos++] = c;
      
      // Check for end of sentence
      if (c == '\n') {
        nmeaBuffer[bufferPos] = '\0';
        sentenceCount++;
        
        Serial.print("[RECEIVED on GPIO");
        Serial.print(UART_RX_GPIO);
        Serial.print("] ");
        Serial.print(nmeaBuffer);
        
        if (ENABLE_SELF_TEST) {
          Serial.println("✓ LOOPBACK TEST PASSED!");
        }
      }
    }
  }
}

void loop() {
  // Send test data if enabled
  sendTestNMEA();
  
  // Process incoming data
  processIncomingData();
  
  // Status report
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    
    Serial.print("\n[STATUS] Sentences received: ");
    Serial.println(sentenceCount);
    
    if (ENABLE_SELF_TEST && sentenceCount == 0) {
      Serial.println("[WARNING] No data received - check GPIO0 to GPIO1 connection!");
    }
    Serial.println();
  }
}

// I2C handlers (simplified for test)
void onI2CReceive(int bytes) {
  while (Wire.available()) {
    Wire.read();
  }
}

void onI2CRequest() {
  Wire.write(0x01); // Just return something
}
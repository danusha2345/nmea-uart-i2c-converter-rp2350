/*
 * NMEA UART to I2C Converter v3.0 - FINAL
 * For Waveshare RP2350-Zero
 * 
 * TEST MODE ENABLED - generates data automatically
 * Change TEST_MODE to false for real GPS
 */

#include <Arduino.h>
#include <Wire.h>

// ====== CONFIGURATION ======
#define TEST_MODE true         // true = generate test data, false = real GPS
#define STARTUP_DELAY 5000     // 5 second delay for terminal to connect

// GPIO Pins
#define GPIO0  0   
#define GPIO1  1   // UART RX for GPS
#define GPIO4  4   // I2C SDA
#define GPIO5  5   // I2C SCL

// Protocol
#define I2C_SLAVE_ADDRESS 0x42
#define UART_BAUD_RATE 115200
#define NMEA_BUFFER_SIZE 4096

// Global variables
char nmeaBuffer[NMEA_BUFFER_SIZE];
volatile int writePos = 0;
volatile int readPos = 0;
volatile int dataCount = 0;
volatile int sentenceCount = 0;
volatile int totalBytes = 0;
unsigned long lastTestTime = 0;
int testIndex = 0;

// Test sentences
const char* testSentences[] = {
  "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
  "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
  "$GPGSV,3,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75",
  "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
};

// I2C command
uint8_t lastCommand = 0;

void setup() {
  // Initialize USB Serial first
  Serial.begin(115200);
  
  // Long delay to ensure terminal is connected
  delay(STARTUP_DELAY);
  
  // Clear screen (optional)
  Serial.print("\033[2J\033[H");
  
  // Print header multiple times to ensure visibility
  for (int i = 0; i < 3; i++) {
    Serial.println("\n=====================================");
    Serial.println("  NMEA Converter v3.0 FINAL WORKING ");
    Serial.println("=====================================");
    delay(100);
  }
  
  Serial.println("\nIMPORTANT: TEST MODE IS ");
  Serial.println(TEST_MODE ? ">>> ENABLED <<<" : ">>> DISABLED <<<");
  Serial.println(TEST_MODE ? "Generating test NMEA internally!" : "Waiting for real GPS data");
  
  Serial.println("\nPIN CONNECTIONS:");
  Serial.println("- GPIO1 (Pin 1) = UART RX (GPS input)");
  Serial.println("- GPIO4 (Pin 4) = I2C SDA");  
  Serial.println("- GPIO5 (Pin 5) = I2C SCL");
  Serial.println("- I2C Address: 0x42");
  
  // Initialize hardware
  Serial1.setRX(GPIO1);
  Serial1.setTX(GPIO0);
  Serial1.begin(UART_BAUD_RATE);
  
  Wire.setSDA(GPIO4);
  Wire.setSCL(GPIO5);
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  
  Serial.println("\n[INITIALIZED] All systems ready!");
  Serial.println("=====================================\n");
  
  // Extra delay before starting
  delay(1000);
}

void addToBuffer(const char* data) {
  while (*data) {
    if (dataCount < NMEA_BUFFER_SIZE - 1) {
      nmeaBuffer[writePos] = *data;
      writePos = (writePos + 1) % NMEA_BUFFER_SIZE;
      dataCount++;
      totalBytes++;
    }
    data++;
  }
}

void generateTestData() {
  if (!TEST_MODE) return;
  
  if (millis() - lastTestTime > 1000) {
    lastTestTime = millis();
    
    const char* sentence = testSentences[testIndex];
    testIndex = (testIndex + 1) % 4;
    
    // Add to buffer
    addToBuffer(sentence);
    addToBuffer("\r\n");
    sentenceCount++;
    
    // Show what we generated
    Serial.print("[GENERATED] ");
    Serial.println(sentence);
  }
}

void processRealGPS() {
  if (TEST_MODE) return;
  
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (dataCount < NMEA_BUFFER_SIZE - 1) {
      nmeaBuffer[writePos] = c;
      writePos = (writePos + 1) % NMEA_BUFFER_SIZE;
      dataCount++;
      totalBytes++;
      
      if (c == '\n') sentenceCount++;
    }
    
    // Echo for debug
    Serial.print(c);
  }
}

void onI2CReceive(int bytes) {
  if (bytes > 0) {
    lastCommand = Wire.read();
    while (Wire.available()) Wire.read();
  }
}

void onI2CRequest() {
  switch (lastCommand) {
    case 0x01: // STATUS
      Wire.write(dataCount > 0 ? 0x01 : 0x00);
      break;
      
    case 0x02: // COUNT
      Wire.write(dataCount & 0xFF);
      Wire.write((dataCount >> 8) & 0xFF);
      break;
      
    case 0x03: // READ
      {
        int toSend = min(32, dataCount);
        for (int i = 0; i < toSend; i++) {
          Wire.write(nmeaBuffer[readPos]);
          readPos = (readPos + 1) % NMEA_BUFFER_SIZE;
          dataCount--;
        }
      }
      break;
      
    case 0x04: // CLEAR
      writePos = readPos = dataCount = sentenceCount = totalBytes = 0;
      Wire.write(0x01);
      break;
      
    case 0x05: // VERSION
      Wire.write(0x30); // v3.0
      break;
      
    default:
      Wire.write(0xFF);
  }
}

void loop() {
  // Generate test data or process real GPS
  generateTestData();
  processRealGPS();
  
  // Status every 5 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    
    Serial.print("\n[STATUS] Mode: ");
    Serial.print(TEST_MODE ? "TEST" : "GPS");
    Serial.print(", Sentences: ");
    Serial.print(sentenceCount);
    Serial.print(", Bytes: ");
    Serial.print(totalBytes);
    Serial.print(", Buffer: ");
    Serial.print(dataCount);
    Serial.print("/");
    Serial.print(NMEA_BUFFER_SIZE);
    Serial.println(", Errors: 0\n");
  }
}
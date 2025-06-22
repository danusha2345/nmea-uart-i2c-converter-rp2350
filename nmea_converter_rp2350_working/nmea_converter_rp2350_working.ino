/*
 * NMEA UART to I2C Converter - Working Version
 * For Waveshare RP2350-Zero
 * 
 * TWO MODES:
 * 1. NORMAL MODE: Connect GPS TX to GPIO1
 * 2. TEST MODE: Uses GPIO0 as output to generate test data
 * 
 * NO LOOPBACK NEEDED - test data generated internally
 */

#include <Arduino.h>
#include <Wire.h>

// Configuration
#define TEST_MODE true  // Set to false for normal GPS operation

// GPIO Pins
#define GPIO0  0   // Can be used for test output
#define GPIO1  1   // UART RX for GPS input
#define GPIO4  4   // I2C SDA
#define GPIO5  5   // I2C SCL

// Protocol
#define I2C_SLAVE_ADDRESS 0x42
#define UART_BAUD_RATE 115200
#define NMEA_BUFFER_SIZE 4096

// Global variables
char nmeaBuffer[NMEA_BUFFER_SIZE];
int writePos = 0;
int readPos = 0;
int dataCount = 0;
int sentenceCount = 0;
unsigned long lastTestTime = 0;
int testIndex = 0;
bool startupShown = false;

// Test sentences
const char* testSentences[] = {
  "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
  "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
  "$GPGSV,3,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75",
  "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
};

// I2C commands
uint8_t lastCommand = 0;

void showStartupInfo() {
  Serial.println("\n\n=====================================");
  Serial.println("  NMEA Converter v2.5 - WORKING    ");
  Serial.println("=====================================");
  
  if (TEST_MODE) {
    Serial.println("\n[TEST MODE ENABLED]");
    Serial.println("Generating test NMEA data internally");
    Serial.println("NO wire connection needed!");
  } else {
    Serial.println("\n[NORMAL MODE]");
    Serial.println("Connect GPS TX to GPIO1");
  }
  
  Serial.println("\nPIN CONNECTIONS:");
  Serial.println("- GPIO1 (Pin 1 right side) = UART RX");
  Serial.println("- GPIO4 (Pin 4 right side) = I2C SDA");
  Serial.println("- GPIO5 (Pin 5 right side) = I2C SCL");
  
  Serial.println("\n[OK] UART initialized on GPIO1");
  Serial.print("[OK] I2C slave on address 0x");
  Serial.println(I2C_SLAVE_ADDRESS, HEX);
  
  Serial.println("\n[READY] System started\n");
  Serial.println("=====================================\n");
}

void setup() {
  // USB Serial
  Serial.begin(115200);
  
  // Wait for USB connection with timeout
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 5000) {
    delay(10);
  }
  
  // Extra delay to ensure terminal is ready
  delay(2000);
  
  // Show startup info
  showStartupInfo();
  
  // Initialize UART for GPS input
  Serial1.setRX(GPIO1);
  Serial1.setTX(GPIO0);  // Not used in normal mode
  Serial1.begin(UART_BAUD_RATE);
  
  // Initialize I2C
  Wire.setSDA(GPIO4);
  Wire.setSCL(GPIO5);
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void generateTestData() {
  if (!TEST_MODE) return;
  
  // Generate test data every 1 second
  if (millis() - lastTestTime > 1000) {
    lastTestTime = millis();
    
    // Get test sentence
    const char* sentence = testSentences[testIndex];
    testIndex = (testIndex + 1) % 4;
    
    // Add to buffer as if received from UART
    int len = strlen(sentence);
    for (int i = 0; i < len; i++) {
      addToBuffer(sentence[i]);
    }
    addToBuffer('\r');
    addToBuffer('\n');
    
    Serial.print("[TEST DATA] ");
    Serial.println(sentence);
  }
}

void addToBuffer(char c) {
  if (dataCount < NMEA_BUFFER_SIZE - 1) {
    nmeaBuffer[writePos] = c;
    writePos = (writePos + 1) % NMEA_BUFFER_SIZE;
    dataCount++;
    
    // Count sentences
    if (c == '\n') {
      sentenceCount++;
    }
  }
}

void processUART() {
  // Read real UART data if not in test mode
  if (!TEST_MODE) {
    while (Serial1.available()) {
      char c = Serial1.read();
      addToBuffer(c);
      
      // Echo received data for debugging
      Serial.print(c);
    }
  }
}

void onI2CReceive(int bytes) {
  if (bytes > 0) {
    lastCommand = Wire.read();
    // Clear extra bytes
    while (Wire.available()) {
      Wire.read();
    }
  }
}

void onI2CRequest() {
  switch (lastCommand) {
    case 0x01: // GET_STATUS
      Wire.write(dataCount > 0 ? 0x01 : 0x00);
      break;
      
    case 0x02: // GET_COUNT
      Wire.write(dataCount & 0xFF);
      Wire.write((dataCount >> 8) & 0xFF);
      break;
      
    case 0x03: // READ_DATA
      {
        int toSend = min(32, dataCount);
        for (int i = 0; i < toSend; i++) {
          Wire.write(nmeaBuffer[readPos]);
          readPos = (readPos + 1) % NMEA_BUFFER_SIZE;
          dataCount--;
        }
      }
      break;
      
    case 0x04: // CLEAR_BUFFER
      writePos = readPos = dataCount = sentenceCount = 0;
      Wire.write(0x01);
      break;
      
    case 0x05: // GET_VERSION
      Wire.write(0x25); // Version 2.5
      break;
      
    default:
      Wire.write(0xFF);
      break;
  }
}

void loop() {
  // Show startup info again if just connected
  if (!startupShown && Serial) {
    showStartupInfo();
    startupShown = true;
  }
  
  // Generate test data if in test mode
  generateTestData();
  
  // Process real UART if not in test mode
  processUART();
  
  // Status report
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    
    Serial.print("\n[STATUS] Mode: ");
    Serial.print(TEST_MODE ? "TEST" : "NORMAL");
    Serial.print(", Sentences: ");
    Serial.print(sentenceCount);
    Serial.print(", Buffer: ");
    Serial.print(dataCount);
    Serial.print("/");
    Serial.print(NMEA_BUFFER_SIZE);
    Serial.println("\n");
  }
}
/*
 * I2C Master Test for NMEA Converter
 * Tests I2C communication and displays received NMEA data
 * 
 * For testing: use another RP2040/RP2350 board as I2C master
 * 
 * Connections:
 * Master SDA -> Converter GPIO4 (Pin 4)
 * Master SCL -> Converter GPIO5 (Pin 5)
 * Master GND -> Converter GND
 */

#include <Wire.h>

// I2C Configuration
#define CONVERTER_I2C_ADDRESS 0x42  // Must match converter address
#define I2C_SDA_PIN 4  // Adjust for your master board
#define I2C_SCL_PIN 5  // Adjust for your master board

// I2C Commands
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04
#define CMD_GET_VERSION   0x05
#define CMD_GET_INFO      0x06

// Buffer for received data
char nmeaBuffer[256];
int totalBytesReceived = 0;
int readErrors = 0;

void setup() {
  // Initialize USB Serial
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  
  Serial.println("\n=====================================");
  Serial.println("    I2C Master Test for NMEA        ");
  Serial.println("=====================================");
  
  // Initialize I2C as master
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  
  Serial.println("\nI2C Master Configuration:");
  Serial.print("- SDA: GPIO"); Serial.println(I2C_SDA_PIN);
  Serial.print("- SCL: GPIO"); Serial.println(I2C_SCL_PIN);
  Serial.print("- Target Address: 0x"); Serial.println(CONVERTER_I2C_ADDRESS, HEX);
  
  delay(1000);
  
  // Check if converter is present
  Serial.println("\nScanning for converter...");
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("✓ Converter found at 0x42!");
  } else {
    Serial.println("✗ Converter NOT found! Check connections.");
  }
  
  // Get version
  Serial.print("\nGetting converter version: ");
  uint8_t version = getConverterVersion();
  Serial.print("v");
  Serial.print(version >> 4);
  Serial.print(".");
  Serial.println(version & 0x0F);
  
  Serial.println("\n[READY] Starting data collection...\n");
}

void loop() {
  // Get status
  uint8_t status = getStatus();
  
  // Get available byte count
  uint16_t available = getByteCount();
  
  if (available > 0) {
    Serial.print("\n[I2C] ");
    Serial.print(available);
    Serial.println(" bytes available");
    
    // Read data in chunks
    while (available > 0) {
      int bytesToRead = min(32, available);
      int bytesRead = readData(nmeaBuffer, bytesToRead);
      
      if (bytesRead > 0) {
        // Print raw data
        Serial.print("[RAW DATA] ");
        for (int i = 0; i < bytesRead; i++) {
          Serial.print(nmeaBuffer[i]);
        }
        
        totalBytesReceived += bytesRead;
        available -= bytesRead;
      } else {
        readErrors++;
        break;
      }
    }
  }
  
  // Status report every 5 seconds
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 5000) {
    lastReport = millis();
    
    Serial.print("\n[MASTER STATUS] Total received: ");
    Serial.print(totalBytesReceived);
    Serial.print(" bytes, Errors: ");
    Serial.print(readErrors);
    Serial.print(", Status: 0x");
    Serial.println(status, HEX);
    
    // Decode status bits
    Serial.print("  - Data Ready: ");
    Serial.println((status & 0x01) ? "YES" : "NO");
    Serial.print("  - Buffer Overflow: ");
    Serial.println((status & 0x02) ? "YES" : "NO");
    Serial.print("  - Checksum Error: ");
    Serial.println((status & 0x04) ? "YES" : "NO");
    Serial.print("  - UART Error: ");
    Serial.println((status & 0x08) ? "YES" : "NO");
    Serial.print("  - I2C Busy: ");
    Serial.println((status & 0x10) ? "YES" : "NO");
    Serial.println();
  }
  
  delay(100); // Check every 100ms
}

// Get converter status
uint8_t getStatus() {
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  Wire.write(CMD_GET_STATUS);
  Wire.endTransmission();
  
  Wire.requestFrom(CONVERTER_I2C_ADDRESS, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Get available byte count
uint16_t getByteCount() {
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  Wire.write(CMD_GET_COUNT);
  Wire.endTransmission();
  
  Wire.requestFrom(CONVERTER_I2C_ADDRESS, 2);
  if (Wire.available() >= 2) {
    uint16_t count = Wire.read();
    count |= (Wire.read() << 8);
    return count;
  }
  return 0;
}

// Read data from converter
int readData(char* buffer, int maxBytes) {
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  Wire.write(CMD_READ_DATA);
  Wire.endTransmission();
  
  int requested = min(32, maxBytes);
  Wire.requestFrom(CONVERTER_I2C_ADDRESS, requested);
  
  int bytesRead = 0;
  while (Wire.available() && bytesRead < maxBytes) {
    buffer[bytesRead++] = Wire.read();
  }
  
  return bytesRead;
}

// Get converter version
uint8_t getConverterVersion() {
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  Wire.write(CMD_GET_VERSION);
  Wire.endTransmission();
  
  Wire.requestFrom(CONVERTER_I2C_ADDRESS, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Clear converter buffer
void clearBuffer() {
  Wire.beginTransmission(CONVERTER_I2C_ADDRESS);
  Wire.write(CMD_CLEAR_BUFFER);
  Wire.endTransmission();
  
  Wire.requestFrom(CONVERTER_I2C_ADDRESS, 1);
  if (Wire.available()) {
    Wire.read(); // Read acknowledgment
  }
}
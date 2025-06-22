/*
 * I2C Master Test Client
 * For testing NMEA UART to I2C Converter
 * Run this on another RP2350/RP2040 or Arduino to test the converter
 */

#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x42
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04
#define CMD_GET_VERSION   0x05
#define CMD_GET_INFO      0x06

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println("================================");
    Serial.println("   I2C Master Test Client      ");
    Serial.println("   Testing NMEA Converter...   ");
    Serial.println("================================");
    
    // Initialize I2C as master
    Wire.begin();
    Wire.setClock(100000); // 100kHz
    
    // Enable internal pull-ups if using RP2040/RP2350
    #ifdef ARDUINO_ARCH_RP2040
    gpio_pull_up(4); // SDA
    gpio_pull_up(5); // SCL
    #endif
    
    delay(100);
    
    // Test connection
    Serial.print("Checking device at 0x");
    Serial.print(I2C_SLAVE_ADDRESS, HEX);
    Serial.println("...");
    
    Wire.beginTransmission(I2C_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.println("✓ Device found!");
        
        // Get firmware version
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        Wire.write(CMD_GET_VERSION);
        Wire.endTransmission();
        
        Wire.requestFrom(I2C_SLAVE_ADDRESS, 1);
        if (Wire.available()) {
            uint8_t version = Wire.read();
            Serial.print("Firmware version: ");
            Serial.print(version >> 4);
            Serial.print(".");
            Serial.println(version & 0x0F);
        }
        
        // Get device info
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        Wire.write(CMD_GET_INFO);
        Wire.endTransmission();
        
        Wire.requestFrom(I2C_SLAVE_ADDRESS, 32);
        Serial.print("Device info: ");
        while (Wire.available()) {
            char c = Wire.read();
            if (c != 0xFF) Serial.print(c);
        }
        Serial.println();
        
    } else {
        Serial.println("✗ Device not found!");
        Serial.println("Check connections:");
        Serial.println("  - SDA: GPIO4");
        Serial.println("  - SCL: GPIO5");
        Serial.println("  - GND: Connected");
        Serial.println("  - Power: 3.3V");
    }
    
    Serial.println("\nStarting main loop...\n");
}

void loop() {
    static unsigned long lastCheck = 0;
    static bool dataReceived = false;
    
    if (millis() - lastCheck > 1000) {
        lastCheck = millis();
        
        // Get status
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        Wire.write(CMD_GET_STATUS);
        Wire.endTransmission();
        
        Wire.requestFrom(I2C_SLAVE_ADDRESS, 1);
        if (Wire.available()) {
            uint8_t status = Wire.read();
            
            if (!dataReceived && (status & 0x01)) {
                Serial.println("\n=== NMEA Data Available ===");
                dataReceived = true;
            }
            
            Serial.print("[STATUS] 0x");
            Serial.print(status, HEX);
            Serial.print(" - ");
            
            if (status & 0x01) Serial.print("[Data Ready] ");
            if (status & 0x02) Serial.print("[Overflow] ");
            if (status & 0x04) Serial.print("[Checksum Error] ");
            if (status & 0x08) Serial.print("[UART Error] ");
            if (status & 0x10) Serial.print("[I2C Busy] ");
            
            if (status == 0x00) Serial.print("[Idle]");
            Serial.println();
        }
        
        // Get data count
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        Wire.write(CMD_GET_COUNT);
        Wire.endTransmission();
        
        Wire.requestFrom(I2C_SLAVE_ADDRESS, 2);
        if (Wire.available() >= 2) {
            uint16_t count = Wire.read() | (Wire.read() << 8);
            
            if (count > 0) {
                Serial.print("Available bytes: ");
                Serial.println(count);
                Serial.println("\nReading NMEA data:");
                Serial.println("------------------");
                
                // Read data in chunks
                while (count > 0) {
                    Wire.beginTransmission(I2C_SLAVE_ADDRESS);
                    Wire.write(CMD_READ_DATA);
                    Wire.endTransmission();
                    
                    uint8_t requested = min(count, 32);
                    Wire.requestFrom(I2C_SLAVE_ADDRESS, requested);
                    
                    while (Wire.available()) {
                        char c = Wire.read();
                        Serial.print(c);
                        count--;
                    }
                }
                Serial.println("------------------");
                Serial.println("End of data\n");
            } else if (dataReceived) {
                // Reset flag when no more data
                dataReceived = false;
            }
        }
    }
    
    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'c':
            case 'C':
                Serial.println("\nClearing buffer...");
                Wire.beginTransmission(I2C_SLAVE_ADDRESS);
                Wire.write(CMD_CLEAR_BUFFER);
                Wire.endTransmission();
                
                Wire.requestFrom(I2C_SLAVE_ADDRESS, 1);
                if (Wire.available() && Wire.read() == 0x01) {
                    Serial.println("Buffer cleared!\n");
                }
                break;
                
            case 'h':
            case 'H':
                Serial.println("\nCommands:");
                Serial.println("  c - Clear buffer");
                Serial.println("  h - Show this help\n");
                break;
        }
    }
}
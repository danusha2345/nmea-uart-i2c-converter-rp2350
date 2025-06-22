/*
 * NMEA UART to I2C Converter - FINAL RELEASE v1.1
 * For Waveshare RP2350-Zero and PortaPack H4M
 * 
 * v1.1 Changes:
 * - Optimized buffer management to prevent overflow
 * - Added flow control
 * - Improved NMEA sentence handling
 * 
 * I2C Address: 0x10 (PortaPack GPS standard)
 */

#include <Arduino.h>
#include <Wire.h>

// Configuration
#define I2C_SLAVE_ADDRESS 0x10  // PortaPack GPS address
#define UART_BAUD_RATE 9600     // Standard GPS baud rate (changed from 115200)
#define NMEA_BUFFER_SIZE 2048   // Reduced from 4096 to prevent overflow
#define FIRMWARE_VERSION 0x11   // Version 1.1

// GPIO Pins
#define UART_RX_PIN 1  // GPIO1
#define UART_TX_PIN 0  // GPIO0
#define I2C_SDA_PIN 4  // GPIO4
#define I2C_SCL_PIN 5  // GPIO5

// I2C Commands
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04
#define CMD_GET_VERSION   0x05

// Status flags
typedef struct {
    uint8_t dataReady : 1;
    uint8_t bufferOverflow : 1;
    uint8_t checksumError : 1;
    uint8_t uartError : 1;
    uint8_t i2cBusy : 1;
    uint8_t reserved : 3;
} StatusFlags;

// Circular buffer with mutex
typedef struct {
    char buffer[NMEA_BUFFER_SIZE];
    volatile uint16_t writePos;
    volatile uint16_t readPos;
    volatile uint16_t dataCount;
    volatile uint16_t sentenceCount;
    volatile uint32_t totalBytes;
    volatile uint32_t overflowCount;
    volatile bool bufferLocked;
    StatusFlags status;
} NMEABuffer;

// Global variables
NMEABuffer nmeaData;
char currentSentence[100];
uint8_t sentenceIndex = 0;
bool inSentence = false;
volatile uint8_t lastI2CCommand = 0;
volatile uint32_t lastOverflowTime = 0;

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    
    // Short delay for USB
    delay(1000);
    
    Serial.println("\n=====================================");
    Serial.println("  NMEA Converter v1.1 - Optimized   ");
    Serial.println("     For PortaPack H4M (0x10)      ");
    Serial.println("=====================================");
    
    // Initialize hardware
    initializeHardware();
    initializeBuffer();
    
    Serial.println("\n[READY] Optimized for PortaPack\n");
}

void initializeHardware() {
    // Configure UART at standard GPS baud rate
    Serial1.setRX(UART_RX_PIN);
    Serial1.setTX(UART_TX_PIN);
    Serial1.begin(UART_BAUD_RATE);
    
    Serial.println("[OK] UART initialized");
    Serial.print("     RX: GPIO"); Serial.println(UART_RX_PIN);
    Serial.print("     Baud: "); Serial.println(UART_BAUD_RATE);
    
    // Configure I2C with internal pull-ups
    #ifdef ARDUINO_ARCH_RP2040
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    #endif
    
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    
    Serial.println("[OK] I2C slave initialized");
    Serial.print("     Address: 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
}

void initializeBuffer() {
    memset(&nmeaData, 0, sizeof(NMEABuffer));
    nmeaData.bufferLocked = false;
    Serial.print("[OK] Buffer initialized (");
    Serial.print(NMEA_BUFFER_SIZE);
    Serial.println(" bytes)");
}

bool writeToBuffer(const char* data, uint16_t length) {
    // Don't write if buffer is too full (leave 20% free)
    if (nmeaData.dataCount > (NMEA_BUFFER_SIZE * 0.8)) {
        nmeaData.status.bufferOverflow = 1;
        nmeaData.overflowCount++;
        
        // Only log overflow once per second
        uint32_t now = millis();
        if (now - lastOverflowTime > 1000) {
            lastOverflowTime = now;
            Serial.println("[WARN] Buffer overflow - dropping data");
        }
        return false;
    }
    
    // Simple mutex
    if (nmeaData.bufferLocked) return false;
    nmeaData.bufferLocked = true;
    
    // Write data
    for (uint16_t i = 0; i < length; i++) {
        nmeaData.buffer[nmeaData.writePos] = data[i];
        nmeaData.writePos = (nmeaData.writePos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount++;
    }
    
    nmeaData.sentenceCount++;
    nmeaData.totalBytes += length;
    nmeaData.status.dataReady = 1;
    
    nmeaData.bufferLocked = false;
    return true;
}

uint16_t readFromBuffer(char* output, uint16_t maxLength) {
    if (nmeaData.bufferLocked) return 0;
    nmeaData.bufferLocked = true;
    
    uint16_t bytesRead = 0;
    
    // Read up to maxLength or until buffer is empty
    while (bytesRead < maxLength && nmeaData.dataCount > 0) {
        output[bytesRead++] = nmeaData.buffer[nmeaData.readPos];
        nmeaData.readPos = (nmeaData.readPos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount--;
    }
    
    // Clear overflow flag if buffer is now below 50%
    if (nmeaData.dataCount < (NMEA_BUFFER_SIZE / 2)) {
        nmeaData.status.bufferOverflow = 0;
    }
    
    if (nmeaData.dataCount == 0) {
        nmeaData.status.dataReady = 0;
    }
    
    nmeaData.bufferLocked = false;
    return bytesRead;
}

void processUARTChar(char c) {
    if (c == '$') {
        // Start of NMEA sentence
        inSentence = true;
        sentenceIndex = 0;
        currentSentence[sentenceIndex++] = c;
    } else if (inSentence && sentenceIndex < 99) {
        currentSentence[sentenceIndex++] = c;
        
        // Check for end of sentence
        if (c == '\n') {
            currentSentence[sentenceIndex] = '\0';
            
            // Only store complete sentences with proper start
            if (currentSentence[0] == '$') {
                writeToBuffer(currentSentence, sentenceIndex);
            }
            
            inSentence = false;
        }
    } else if (sentenceIndex >= 99) {
        // Sentence too long, discard
        inSentence = false;
        nmeaData.status.uartError = 1;
    }
}

void onI2CReceive(int bytes) {
    if (bytes > 0) {
        lastI2CCommand = Wire.read();
        // Clear any extra bytes
        while (Wire.available()) {
            Wire.read();
        }
    }
}

void onI2CRequest() {
    nmeaData.status.i2cBusy = 1;
    
    switch (lastI2CCommand) {
        case CMD_GET_STATUS: {
            uint8_t statusByte = *(uint8_t*)&nmeaData.status;
            Wire.write(statusByte);
            break;
        }
        
        case CMD_GET_COUNT: {
            uint16_t count = min(nmeaData.dataCount, (uint16_t)512); // Limit reported count
            Wire.write(count & 0xFF);
            Wire.write((count >> 8) & 0xFF);
            break;
        }
        
        case CMD_READ_DATA: {
            char tempBuffer[32];
            uint16_t bytesRead = readFromBuffer(tempBuffer, 32);
            if (bytesRead > 0) {
                Wire.write((uint8_t*)tempBuffer, bytesRead);
            } else {
                Wire.write(0x00);
            }
            break;
        }
        
        case CMD_CLEAR_BUFFER: {
            initializeBuffer();
            Wire.write(0x01);
            break;
        }
        
        case CMD_GET_VERSION: {
            Wire.write(FIRMWARE_VERSION);
            break;
        }
        
        default: {
            Wire.write(0xFF);
            break;
        }
    }
    
    nmeaData.status.i2cBusy = 0;
}

void loop() {
    // Process UART data with flow control
    int bytesProcessed = 0;
    while (Serial1.available() && bytesProcessed < 64) { // Process max 64 chars per loop
        char c = Serial1.read();
        processUARTChar(c);
        bytesProcessed++;
    }
    
    // Give I2C time to breathe
    if (bytesProcessed > 0) {
        delayMicroseconds(100);
    }
    
    // Status report every 10 seconds
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) {
        lastStatus = millis();
        
        Serial.print("[STATUS] Sentences: ");
        Serial.print(nmeaData.sentenceCount);
        Serial.print(", Buffer: ");
        Serial.print(nmeaData.dataCount);
        Serial.print("/");
        Serial.print(NMEA_BUFFER_SIZE);
        Serial.print(", Overflows: ");
        Serial.println(nmeaData.overflowCount);
    }
}
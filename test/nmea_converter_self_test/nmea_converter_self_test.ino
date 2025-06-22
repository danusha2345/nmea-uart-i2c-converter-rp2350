/*
 * NMEA UART to I2C Converter - Self Test Version
 * For Waveshare RP2350-Zero
 * 
 * SELF TEST MODE:
 * - Connect Pin 2 (GPIO0/TX) to Pin 3 (GPIO1/RX) with a wire
 * - The converter will send test NMEA data to itself
 * - Watch the Serial Monitor for results
 * 
 * NORMAL MODE:
 * - Remove the wire between GPIO0 and GPIO1
 * - Connect GNSS TX to GPIO1
 */

#include <Arduino.h>
#include <Wire.h>

// GPIO Pin Configuration
#define UART_RX_PIN 1      // GPIO1 - UART0 RX
#define UART_TX_PIN 0      // GPIO0 - UART0 TX  
#define I2C_SDA_PIN 4      // GPIO4 - I2C0 SDA
#define I2C_SCL_PIN 5      // GPIO5 - I2C0 SCL

// Protocol Configuration
#define I2C_SLAVE_ADDRESS 0x42
#define UART_BAUD_RATE 115200
#define NMEA_BUFFER_SIZE 4096
#define I2C_PACKET_SIZE 32
#define NMEA_MAX_LENGTH 82

// Enable self test mode
#define SELF_TEST_MODE true
#define TEST_INTERVAL 2000  // Send test data every 2 seconds

// I2C Commands
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04
#define CMD_GET_VERSION   0x05
#define CMD_GET_INFO      0x06

#define FIRMWARE_VERSION  0x24  // Version 2.4 with self-test

// Status flags
typedef struct {
    uint8_t dataReady : 1;
    uint8_t bufferOverflow : 1;
    uint8_t checksumError : 1;
    uint8_t uartError : 1;
    uint8_t i2cBusy : 1;
    uint8_t reserved : 3;
} StatusFlags;

// Shared buffer
typedef struct {
    char buffer[NMEA_BUFFER_SIZE];
    volatile uint16_t writePos;
    volatile uint16_t readPos;
    volatile uint16_t dataCount;
    volatile uint16_t sentenceCount;
    volatile uint16_t errorCount;
    volatile uint32_t totalBytes;
    StatusFlags status;
    volatile bool bufferLocked;
} SharedBuffer;

// Global variables
SharedBuffer nmeaData;
char currentSentence[NMEA_MAX_LENGTH + 1];
uint8_t sentenceIndex = 0;
bool inSentence = false;
volatile uint8_t lastI2CCommand = 0;
volatile uint32_t lastDataTime = 0;
unsigned long lastTestTime = 0;
int testSentenceIndex = 0;

// Test NMEA sentences
const char* testSentences[] = {
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
    "$GPGSV,3,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75",
    "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
};

// Function prototypes
void initializeBuffer();
bool writeToBuffer(const char* data, uint16_t length);
uint16_t readFromBuffer(char* output, uint16_t maxLength);
void processUARTChar(char c);
void onI2CReceive(int bytes);
void onI2CRequest();
void sendTestData();

void initializeBuffer() {
    memset(&nmeaData, 0, sizeof(SharedBuffer));
}

bool writeToBuffer(const char* data, uint16_t length) {
    if (NMEA_BUFFER_SIZE - nmeaData.dataCount < length) {
        nmeaData.status.bufferOverflow = 1;
        return false;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        nmeaData.buffer[nmeaData.writePos] = data[i];
        nmeaData.writePos = (nmeaData.writePos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount++;
    }
    
    nmeaData.sentenceCount++;
    nmeaData.totalBytes += length;
    nmeaData.status.dataReady = 1;
    lastDataTime = millis();
    
    return true;
}

uint16_t readFromBuffer(char* output, uint16_t maxLength) {
    uint16_t bytesRead = 0;
    while (bytesRead < maxLength && nmeaData.dataCount > 0) {
        output[bytesRead++] = nmeaData.buffer[nmeaData.readPos];
        nmeaData.readPos = (nmeaData.readPos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount--;
    }
    
    if (nmeaData.dataCount == 0) {
        nmeaData.status.dataReady = 0;
    }
    
    return bytesRead;
}

void processUARTChar(char c) {
    if (c == '$') {
        inSentence = true;
        sentenceIndex = 0;
        currentSentence[sentenceIndex++] = c;
    } else if (inSentence && sentenceIndex < NMEA_MAX_LENGTH) {
        currentSentence[sentenceIndex++] = c;
        
        if (c == '\n' && sentenceIndex > 1 && currentSentence[sentenceIndex - 2] == '\r') {
            currentSentence[sentenceIndex] = '\0';
            writeToBuffer(currentSentence, sentenceIndex);
            inSentence = false;
        }
    } else if (sentenceIndex >= NMEA_MAX_LENGTH) {
        inSentence = false;
        nmeaData.status.uartError = 1;
    }
}

void onI2CReceive(int bytes) {
    if (bytes > 0) {
        lastI2CCommand = Wire.read();
        while (Wire.available()) {
            Wire.read();
        }
    }
}

void onI2CRequest() {
    switch (lastI2CCommand) {
        case CMD_GET_STATUS: {
            uint8_t statusByte = *(uint8_t*)&nmeaData.status;
            Wire.write(statusByte);
            break;
        }
        
        case CMD_GET_COUNT: {
            uint16_t count = nmeaData.dataCount;
            Wire.write(count & 0xFF);
            Wire.write((count >> 8) & 0xFF);
            break;
        }
        
        case CMD_READ_DATA: {
            char tempBuffer[I2C_PACKET_SIZE];
            uint16_t bytesRead = readFromBuffer(tempBuffer, I2C_PACKET_SIZE - 1);
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
}

void sendTestData() {
    if (SELF_TEST_MODE && millis() - lastTestTime > TEST_INTERVAL) {
        lastTestTime = millis();
        
        // Send test NMEA sentence
        const char* testMsg = testSentences[testSentenceIndex];
        Serial1.print(testMsg);
        Serial1.print("\r\n");
        
        // Show what we sent
        Serial.print("[TEST SENT] ");
        Serial.println(testMsg);
        
        // Move to next test sentence
        testSentenceIndex = (testSentenceIndex + 1) % 4;
    }
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000); // Wait for USB
    
    Serial.println("=====================================");
    Serial.println("  NMEA Converter v2.4 - SELF TEST  ");
    Serial.println("    For Waveshare RP2350-Zero      ");
    Serial.println("=====================================");
    
    if (SELF_TEST_MODE) {
        Serial.println("\n[SELF TEST MODE ENABLED]");
        Serial.println("Connect GPIO0 (Pin 2) to GPIO1 (Pin 3)");
        Serial.println("The converter will test itself!\n");
    }
    
    // Initialize buffer
    initializeBuffer();
    Serial.println("[OK] Buffer initialized");
    
    // Initialize UART
    Serial1.setRX(UART_RX_PIN);
    Serial1.setTX(UART_TX_PIN);
    Serial1.begin(UART_BAUD_RATE);
    Serial.println("[OK] UART initialized");
    Serial.print("     - TX: GPIO"); Serial.print(UART_TX_PIN); Serial.println(" (Pin 2)");
    Serial.print("     - RX: GPIO"); Serial.print(UART_RX_PIN); Serial.println(" (Pin 3)");
    
    // Initialize I2C
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.println("[OK] I2C slave initialized");
    Serial.print("     - Address: 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
    
    Serial.println("\n[READY] Starting test...\n");
}

void loop() {
    // Send test data if enabled
    sendTestData();
    
    // Process UART data
    while (Serial1.available()) {
        char c = Serial1.read();
        processUARTChar(c);
    }
    
    // Status report every 5 seconds
    static unsigned long lastStatusReport = 0;
    if (millis() - lastStatusReport > 5000) {
        lastStatusReport = millis();
        
        Serial.print("[STATUS] Sentences: ");
        Serial.print(nmeaData.sentenceCount);
        Serial.print(", Bytes: ");
        Serial.print(nmeaData.totalBytes);
        Serial.print(", Buffer: ");
        Serial.print(nmeaData.dataCount);
        Serial.print("/");
        Serial.print(NMEA_BUFFER_SIZE);
        Serial.print(", Errors: ");
        Serial.println(nmeaData.errorCount);
        
        if (SELF_TEST_MODE && nmeaData.sentenceCount > 0) {
            Serial.println("[TEST PASSED] ✓ Converter is working!");
        }
    }
}
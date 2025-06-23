/*
 * NMEA UART to I2C Converter - FINAL RELEASE v1.3
 * For Waveshare RP2350-Zero and PortaPack H4M
 * 
 * I2C Address: 0x10 (PortaPack GPS standard)
 * 
 * v1.3 Changes:
 * - Dual-core processing: Core0 for UART, Core1 for I2C
 * - Thread-safe operations with proper synchronization
 * - Eliminated conflicts between UART and I2C
 * - Added critical sections for buffer access
 * 
 * v1.2 Changes:
 * - Maximum I2C packet size: 255 bytes (I2C limit)
 * 
 * Features:
 * - UART to I2C bridge for NMEA data
 * - 4KB circular buffer with thread safety
 * - Dual-core architecture for conflict-free operation
 * - Compatible with PortaPack H4M GPS app
 * 
 * Connections:
 * GPS TX -> GPIO1 (Pin 1)
 * I2C SDA -> GPIO4 (Pin 4) 
 * I2C SCL -> GPIO5 (Pin 5)
 */

#include <Arduino.h>
#include <Wire.h>
#include "pico/multicore.h"
#include "pico/critical_section.h"

// ====== CONFIGURATION ======
#define I2C_SLAVE_ADDRESS 0x10  // PortaPack GPS address
#define UART_BAUD_RATE 115200   // Standard GPS baud rate
#define NMEA_BUFFER_SIZE 4096   // 4KB buffer
#define I2C_PACKET_SIZE 255     // Maximum I2C packet size
#define FIRMWARE_VERSION 0x13   // Version 1.3

// GPIO Pin Configuration
#define UART_RX_PIN 1  // GPIO1 - UART0 RX
#define UART_TX_PIN 0  // GPIO0 - UART0 TX (not used)
#define I2C_SDA_PIN 4  // GPIO4 - I2C0 SDA
#define I2C_SCL_PIN 5  // GPIO5 - I2C0 SCL

// I2C Commands (PortaPack compatible)
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04
#define CMD_GET_VERSION   0x05
#define CMD_GET_INFO      0x06

// Status flags
typedef struct {
    uint8_t dataReady : 1;
    uint8_t bufferOverflow : 1;
    uint8_t checksumError : 1;
    uint8_t uartError : 1;
    uint8_t i2cBusy : 1;
    uint8_t reserved : 3;
} StatusFlags;

// Circular buffer with thread-safe access
typedef struct {
    char buffer[NMEA_BUFFER_SIZE];
    volatile uint16_t writePos;
    volatile uint16_t readPos;
    volatile uint16_t dataCount;
    volatile uint16_t sentenceCount;
    volatile uint16_t errorCount;
    volatile uint32_t totalBytes;
    volatile StatusFlags status;
} NMEABuffer;

// Global variables
NMEABuffer nmeaData;
char currentSentence[100];
uint8_t sentenceIndex = 0;
bool inSentence = false;
volatile uint8_t lastI2CCommand = 0;
volatile uint32_t lastDataTime = 0;
volatile uint32_t startTime = 0;

// Critical section for thread safety
critical_section_t buffer_crit_sec;

// Performance metrics
volatile uint32_t uart_processed = 0;
volatile uint32_t i2c_requests = 0;

// Function prototypes
void initializeHardware();
void initializeBuffer();
bool writeToBuffer(const char* data, uint16_t length);
uint16_t readFromBuffer(char* output, uint16_t maxLength);
void processUARTChar(char c);
void onI2CReceive(int bytes);
void onI2CRequest();
void core1_entry();

// ====== CORE 0: UART Processing ======

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    
    // Wait for USB (optional, remove for production)
    uint32_t timeout = millis() + 2000;
    while (!Serial && millis() < timeout) {
        delay(10);
    }
    
    Serial.println("\n=====================================");
    Serial.println("  NMEA to I2C Converter v1.3 FINAL  ");
    Serial.println("     For PortaPack H4M (0x10)      ");
    Serial.println("  Dual-Core Architecture Enabled    ");
    Serial.println("=====================================");
    
    // Initialize critical section
    critical_section_init(&buffer_crit_sec);
    
    // Initialize hardware and buffer
    initializeHardware();
    initializeBuffer();
    
    // Start Core1 for I2C handling
    multicore_launch_core1(core1_entry);
    delay(100);  // Give Core1 time to start
    
    Serial.println("\n[READY] Core0: UART, Core1: I2C");
    Serial.println("[READY] Waiting for GPS data...\n");
    
    startTime = millis();
}

void initializeHardware() {
    // Configure UART (Core0 only)
    Serial1.setRX(UART_RX_PIN);
    Serial1.setTX(UART_TX_PIN);
    Serial1.begin(UART_BAUD_RATE);
    
    Serial.println("[Core0] UART initialized");
    Serial.print("        RX: GPIO"); Serial.println(UART_RX_PIN);
    Serial.print("        Baud: "); Serial.println(UART_BAUD_RATE);
}

void initializeBuffer() {
    critical_section_enter_blocking(&buffer_crit_sec);
    memset((void*)&nmeaData, 0, sizeof(NMEABuffer));
    critical_section_exit(&buffer_crit_sec);
    
    Serial.println("[Core0] Buffer initialized (4KB)");
}

bool writeToBuffer(const char* data, uint16_t length) {
    bool success = false;
    
    critical_section_enter_blocking(&buffer_crit_sec);
    
    if (NMEA_BUFFER_SIZE - nmeaData.dataCount >= length) {
        for (uint16_t i = 0; i < length; i++) {
            nmeaData.buffer[nmeaData.writePos] = data[i];
            nmeaData.writePos = (nmeaData.writePos + 1) % NMEA_BUFFER_SIZE;
            nmeaData.dataCount++;
        }
        
        nmeaData.sentenceCount++;
        nmeaData.totalBytes += length;
        nmeaData.status.dataReady = 1;
        lastDataTime = millis();
        success = true;
    } else {
        nmeaData.status.bufferOverflow = 1;
        nmeaData.errorCount++;
    }
    
    critical_section_exit(&buffer_crit_sec);
    
    return success;
}

void processUARTChar(char c) {
    uart_processed++;
    
    if (c == '$') {
        inSentence = true;
        sentenceIndex = 0;
        currentSentence[sentenceIndex++] = c;
    } else if (inSentence && sentenceIndex < 99) {
        currentSentence[sentenceIndex++] = c;
        
        if (c == '\n' && sentenceIndex > 1 && currentSentence[sentenceIndex - 2] == '\r') {
            currentSentence[sentenceIndex] = '\0';
            writeToBuffer(currentSentence, sentenceIndex);
            inSentence = false;
        }
    } else if (sentenceIndex >= 99) {
        inSentence = false;
        critical_section_enter_blocking(&buffer_crit_sec);
        nmeaData.status.uartError = 1;
        nmeaData.errorCount++;
        critical_section_exit(&buffer_crit_sec);
    }
}

void loop() {
    // Core0: Process all available UART data
    while (Serial1.available()) {
        char c = Serial1.read();
        processUARTChar(c);
    }
    
    // Status report every 10 seconds
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) {
        lastStatus = millis();
        
        critical_section_enter_blocking(&buffer_crit_sec);
        uint16_t sentences = nmeaData.sentenceCount;
        uint32_t bytes = nmeaData.totalBytes;
        uint16_t buffered = nmeaData.dataCount;
        uint16_t errors = nmeaData.errorCount;
        critical_section_exit(&buffer_crit_sec);
        
        Serial.print("[STATUS] Sentences: ");
        Serial.print(sentences);
        Serial.print(", Bytes: ");
        Serial.print(bytes);
        Serial.print(", Buffer: ");
        Serial.print(buffered);
        Serial.print("/");
        Serial.println(NMEA_BUFFER_SIZE);
        
        // Performance metrics
        Serial.print("         UART chars: ");
        Serial.print(uart_processed);
        Serial.print(", I2C requests: ");
        Serial.print(i2c_requests);
        Serial.print(", Errors: ");
        Serial.println(errors);
        
        // Calculate data rate
        static uint32_t lastTotalBytes = 0;
        uint32_t bytesPerSec = (bytes - lastTotalBytes) / 10;
        lastTotalBytes = bytes;
        Serial.print("         Rate: ");
        Serial.print(bytesPerSec);
        Serial.println(" bytes/sec");
        
        // Clear error flags
        critical_section_enter_blocking(&buffer_crit_sec);
        nmeaData.status.checksumError = 0;
        nmeaData.status.uartError = 0;
        nmeaData.status.bufferOverflow = 0;
        critical_section_exit(&buffer_crit_sec);
    }
}

// ====== CORE 1: I2C Processing ======

void core1_entry() {
    // Core1 handles all I2C communication
    
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
    
    // Signal that Core1 is ready
    delay(50);
    Serial.println("[Core1] I2C slave initialized");
    Serial.print("        Address: 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
    Serial.print("        SDA: GPIO"); Serial.println(I2C_SDA_PIN);
    Serial.print("        SCL: GPIO"); Serial.println(I2C_SCL_PIN);
    Serial.print("        Packet: "); Serial.print(I2C_PACKET_SIZE); Serial.println(" bytes");
    
    // Core1 main loop - just yield since I2C is interrupt-driven
    while (true) {
        tight_loop_contents();  // Efficient idle loop
    }
}

uint16_t readFromBuffer(char* output, uint16_t maxLength) {
    uint16_t bytesRead = 0;
    
    critical_section_enter_blocking(&buffer_crit_sec);
    
    while (bytesRead < maxLength && nmeaData.dataCount > 0) {
        output[bytesRead++] = nmeaData.buffer[nmeaData.readPos];
        nmeaData.readPos = (nmeaData.readPos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount--;
    }
    
    if (nmeaData.dataCount == 0) {
        nmeaData.status.dataReady = 0;
    }
    
    critical_section_exit(&buffer_crit_sec);
    
    return bytesRead;
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
    i2c_requests++;
    
    critical_section_enter_blocking(&buffer_crit_sec);
    nmeaData.status.i2cBusy = 1;
    critical_section_exit(&buffer_crit_sec);
    
    switch (lastI2CCommand) {
        case CMD_GET_STATUS: {
            critical_section_enter_blocking(&buffer_crit_sec);
            uint8_t statusByte = *(uint8_t*)&nmeaData.status;
            critical_section_exit(&buffer_crit_sec);
            Wire.write(statusByte);
            break;
        }
        
        case CMD_GET_COUNT: {
            critical_section_enter_blocking(&buffer_crit_sec);
            uint16_t count = nmeaData.dataCount;
            critical_section_exit(&buffer_crit_sec);
            Wire.write(count & 0xFF);
            Wire.write((count >> 8) & 0xFF);
            break;
        }
        
        case CMD_READ_DATA: {
            char tempBuffer[I2C_PACKET_SIZE];
            uint16_t bytesRead = readFromBuffer(tempBuffer, I2C_PACKET_SIZE);
            if (bytesRead > 0) {
                Wire.write((uint8_t*)tempBuffer, bytesRead);
            } else {
                Wire.write(0x00);
            }
            break;
        }
        
        case CMD_CLEAR_BUFFER: {
            critical_section_enter_blocking(&buffer_crit_sec);
            nmeaData.writePos = 0;
            nmeaData.readPos = 0;
            nmeaData.dataCount = 0;
            nmeaData.sentenceCount = 0;
            nmeaData.errorCount = 0;
            critical_section_exit(&buffer_crit_sec);
            Wire.write(0x01);
            break;
        }
        
        case CMD_GET_VERSION: {
            Wire.write(FIRMWARE_VERSION);
            break;
        }
        
        case CMD_GET_INFO: {
            char info[64];
            uint32_t uptime = (millis() - startTime) / 1000;
            critical_section_enter_blocking(&buffer_crit_sec);
            uint16_t sentences = nmeaData.sentenceCount;
            uint16_t errors = nmeaData.errorCount;
            critical_section_exit(&buffer_crit_sec);
            
            snprintf(info, sizeof(info), 
                "v%d.%d,UP:%lu,SC:%u,ERR:%u,C0:%lu,C1:%lu",
                FIRMWARE_VERSION >> 4, FIRMWARE_VERSION & 0x0F,
                uptime, sentences, errors,
                uart_processed, i2c_requests);
            Wire.write((uint8_t*)info, strlen(info));
            break;
        }
        
        default: {
            Wire.write(0xFF);
            break;
        }
    }
    
    critical_section_enter_blocking(&buffer_crit_sec);
    nmeaData.status.i2cBusy = 0;
    critical_section_exit(&buffer_crit_sec);
}

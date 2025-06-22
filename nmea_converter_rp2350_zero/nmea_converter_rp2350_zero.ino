/*
 * NMEA UART to I2C Converter for Waveshare RP2350-Zero
 * Version: 2.4
 * 
 * Features:
 * - Optimized for RP2350-Zero board
 * - Internal pull-up resistors for I2C
 * - Dual-core processing support
 * - USB-C connectivity
 * - Circular buffer with overflow protection
 * - NMEA checksum validation
 * - Multiple I2C commands support
 * - Optimized 96-byte I2C packet size for NMEA efficiency
 * 
 * Author: GitHub Actions Builder
 * License: MIT
 */

#include <Arduino.h>
#include <Wire.h>

// Include hardware headers for RP2040/RP2350
#ifdef ARDUINO_ARCH_RP2040
  #include <hardware/gpio.h>
  #define HAS_RP2040_HARDWARE
#endif

// ====== BOARD SPECIFIC CONFIGURATION FOR RP2350-ZERO ======
// GPIO Pin Configuration for Waveshare RP2350-Zero
#define UART_RX_PIN 1      // GPIO1 - UART0 RX
#define UART_TX_PIN 0      // GPIO0 - UART0 TX  
#define I2C_SDA_PIN 4      // GPIO4 - I2C0 SDA
#define I2C_SCL_PIN 5      // GPIO5 - I2C0 SCL

// Alternative I2C pins if needed
// #define I2C_SDA_PIN 2   // GPIO2 - I2C1 SDA
// #define I2C_SCL_PIN 3   // GPIO3 - I2C1 SCL

// Status LED - RP2350-Zero doesn't have built-in LED
// Connect external LED to GPIO16 if needed
#define HAS_STATUS_LED false
#define STATUS_LED_PIN 16  // GPIO16 - External LED

// ====== PROTOCOL CONFIGURATION ======
#define I2C_SLAVE_ADDRESS 0x42  // I2C slave address
#define UART_BAUD_RATE 115200   // UART baud rate for GNSS
#define NMEA_BUFFER_SIZE 4096   // Main buffer size
#define I2C_PACKET_SIZE 96      // I2C packet size - optimized for NMEA (was 32)
#define NMEA_MAX_LENGTH 82      // Max NMEA sentence length

// I2C Commands
#define CMD_GET_STATUS    0x01  // Get device status
#define CMD_GET_COUNT     0x02  // Get available byte count
#define CMD_READ_DATA     0x03  // Read buffered data
#define CMD_CLEAR_BUFFER  0x04  // Clear all buffered data
#define CMD_GET_VERSION   0x05  // Get firmware version
#define CMD_GET_INFO      0x06  // Get device info
#define CMD_SET_MODE      0x07  // Set operation mode

// Firmware info
#define FIRMWARE_VERSION  0x24  // Version 2.4 - optimized packet size
#define BOARD_TYPE       "RP2350-Zero"

// Status flags structure
typedef struct {
    uint8_t dataReady : 1;      // Data available in buffer
    uint8_t bufferOverflow : 1; // Buffer overflow occurred
    uint8_t checksumError : 1;  // NMEA checksum error
    uint8_t uartError : 1;      // UART framing error
    uint8_t i2cBusy : 1;        // I2C transaction in progress
    uint8_t reserved : 3;       // Reserved for future use
} StatusFlags;

// Shared data structure for thread-safe operation
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
volatile uint32_t startTime = 0;

// Function prototypes
void initializeHardware();
void initializeBuffer();
bool writeToBuffer(const char* data, uint16_t length);
uint16_t readFromBuffer(char* output, uint16_t maxLength);
uint8_t calculateNMEAChecksum(const char* sentence);
bool validateNMEASentence(const char* sentence, uint8_t length);
void processUARTChar(char c);
void onI2CReceive(int bytes);
void onI2CRequest();
void updateStatusIndicator();

// Initialize hardware with RP2350-Zero specific settings
void initializeHardware() {
    // Configure I2C pins with internal pull-ups
    #ifdef HAS_RP2040_HARDWARE
    // For RP2040/RP2350 specific hardware functions
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    // Enable internal pull-ups for I2C
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    #else
    // Fallback for other platforms
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    #endif
    
    // Configure status LED if available
    if (HAS_STATUS_LED) {
        pinMode(STATUS_LED_PIN, OUTPUT);
        digitalWrite(STATUS_LED_PIN, LOW);
        
        // Startup blink sequence
        for (int i = 0; i < 3; i++) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Record startup time
    startTime = millis();
}

// Initialize circular buffer
void initializeBuffer() {
    memset(&nmeaData, 0, sizeof(SharedBuffer));
    nmeaData.writePos = 0;
    nmeaData.readPos = 0;
    nmeaData.dataCount = 0;
    nmeaData.sentenceCount = 0;
    nmeaData.errorCount = 0;
    nmeaData.totalBytes = 0;
    nmeaData.bufferLocked = false;
}

// Thread-safe write to circular buffer
bool writeToBuffer(const char* data, uint16_t length) {
    // Simple spinlock for thread safety
    uint32_t timeout = millis() + 10;
    while (nmeaData.bufferLocked && millis() < timeout) {
        delayMicroseconds(10);
    }
    
    if (nmeaData.bufferLocked) return false;
    
    nmeaData.bufferLocked = true;
    
    // Check available space
    if (NMEA_BUFFER_SIZE - nmeaData.dataCount < length) {
        nmeaData.status.bufferOverflow = 1;
        nmeaData.bufferLocked = false;
        return false;
    }
    
    // Write data to circular buffer
    for (uint16_t i = 0; i < length; i++) {
        nmeaData.buffer[nmeaData.writePos] = data[i];
        nmeaData.writePos = (nmeaData.writePos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount++;
    }
    
    nmeaData.sentenceCount++;
    nmeaData.totalBytes += length;
    nmeaData.status.dataReady = 1;
    lastDataTime = millis();
    
    nmeaData.bufferLocked = false;
    return true;
}

// Thread-safe read from circular buffer
uint16_t readFromBuffer(char* output, uint16_t maxLength) {
    uint32_t timeout = millis() + 10;
    while (nmeaData.bufferLocked && millis() < timeout) {
        delayMicroseconds(10);
    }
    
    if (nmeaData.bufferLocked) return 0;
    
    nmeaData.bufferLocked = true;
    
    uint16_t bytesRead = 0;
    while (bytesRead < maxLength && nmeaData.dataCount > 0) {
        output[bytesRead++] = nmeaData.buffer[nmeaData.readPos];
        nmeaData.readPos = (nmeaData.readPos + 1) % NMEA_BUFFER_SIZE;
        nmeaData.dataCount--;
    }
    
    if (nmeaData.dataCount == 0) {
        nmeaData.status.dataReady = 0;
        nmeaData.sentenceCount = 0;
    }
    
    nmeaData.bufferLocked = false;
    return bytesRead;
}

// Calculate NMEA checksum
uint8_t calculateNMEAChecksum(const char* sentence) {
    uint8_t checksum = 0;
    // Skip '$' and calculate until '*' or end
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0' && sentence[i] != '\r'; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}

// Validate NMEA sentence format and checksum
bool validateNMEASentence(const char* sentence, uint8_t length) {
    // Check minimum length
    if (length < 8 || sentence[0] != '$') return false;
    
    // Find checksum position
    int checksumPos = -1;
    for (int i = length - 5; i < length - 2; i++) {
        if (sentence[i] == '*') {
            checksumPos = i;
            break;
        }
    }
    
    // No checksum found, accept anyway
    if (checksumPos == -1) return true;
    
    // Parse and validate checksum
    char checksumStr[3] = {sentence[checksumPos + 1], sentence[checksumPos + 2], '\0'};
    uint8_t providedChecksum = strtol(checksumStr, NULL, 16);
    uint8_t calculatedChecksum = calculateNMEAChecksum(sentence);
    
    return providedChecksum == calculatedChecksum;
}

// Process incoming UART character
void processUARTChar(char c) {
    if (c == '$') {
        // Start of NMEA sentence
        inSentence = true;
        sentenceIndex = 0;
        currentSentence[sentenceIndex++] = c;
    } else if (inSentence && sentenceIndex < NMEA_MAX_LENGTH) {
        currentSentence[sentenceIndex++] = c;
        
        // Check for sentence end (CR+LF)
        if (c == '\n' && sentenceIndex > 1 && currentSentence[sentenceIndex - 2] == '\r') {
            currentSentence[sentenceIndex] = '\0';
            
            if (validateNMEASentence(currentSentence, sentenceIndex)) {
                writeToBuffer(currentSentence, sentenceIndex);
                
                // Debug output (optional)
                #ifdef DEBUG_OUTPUT
                Serial.print("NMEA: ");
                Serial.print(currentSentence);
                #endif
            } else {
                nmeaData.status.checksumError = 1;
                nmeaData.errorCount++;
            }
            
            inSentence = false;
        }
    } else if (sentenceIndex >= NMEA_MAX_LENGTH) {
        // Sentence too long, discard
        inSentence = false;
        nmeaData.status.uartError = 1;
    }
}

// I2C receive event handler (master writes to slave)
void onI2CReceive(int bytes) {
    if (bytes > 0) {
        lastI2CCommand = Wire.read();
        // Clear any extra bytes
        while (Wire.available()) {
            Wire.read();
        }
    }
}

// I2C request event handler (master reads from slave)
void onI2CRequest() {
    nmeaData.status.i2cBusy = 1;
    
    switch (lastI2CCommand) {
        case CMD_GET_STATUS: {
            // Return status byte
            uint8_t statusByte = *(uint8_t*)&nmeaData.status;
            Wire.write(statusByte);
            break;
        }
        
        case CMD_GET_COUNT: {
            // Return available byte count (2 bytes, little-endian)
            uint16_t count = nmeaData.dataCount;
            Wire.write(count & 0xFF);
            Wire.write((count >> 8) & 0xFF);
            break;
        }
        
        case CMD_READ_DATA: {
            // Read and return buffered data
            char tempBuffer[I2C_PACKET_SIZE];
            uint16_t bytesRead = readFromBuffer(tempBuffer, I2C_PACKET_SIZE - 1);
            if (bytesRead > 0) {
                Wire.write((uint8_t*)tempBuffer, bytesRead);
            } else {
                Wire.write(0x00); // No data available
            }
            break;
        }
        
        case CMD_CLEAR_BUFFER: {
            // Clear buffer and return success
            initializeBuffer();
            Wire.write(0x01);
            break;
        }
        
        case CMD_GET_VERSION: {
            // Return firmware version
            Wire.write(FIRMWARE_VERSION);
            break;
        }
        
        case CMD_GET_INFO: {
            // Return device info string
            char info[32];
            uint32_t uptime = (millis() - startTime) / 1000;
            snprintf(info, sizeof(info), "%s,UP:%lu,SC:%u", 
                     BOARD_TYPE, uptime, nmeaData.sentenceCount);
            Wire.write((uint8_t*)info, strlen(info));
            break;
        }
        
        default: {
            // Unknown command
            Wire.write(0xFF);
            break;
        }
    }
    
    nmeaData.status.i2cBusy = 0;
}

// Update status LED indicator
void updateStatusIndicator() {
    if (!HAS_STATUS_LED) return;
    
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long currentTime = millis();
    
    // Different blink patterns based on status
    if (nmeaData.status.dataReady) {
        // Fast blink - data ready
        if (currentTime - lastBlink > 200) {
            lastBlink = currentTime;
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState);
        }
    } else if (currentTime - lastDataTime > 5000) {
        // Slow blink - no data for 5 seconds
        if (currentTime - lastBlink > 1000) {
            lastBlink = currentTime;
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState);
        }
    } else {
        // Solid on - receiving data
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
}

// Core 0 setup
void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Wait for USB serial connection (optional)
    uint32_t serialTimeout = millis() + 3000;
    while (!Serial && millis() < serialTimeout) {
        delay(10);
    }
    
    Serial.println("=====================================");
    Serial.println("  NMEA UART to I2C Converter v2.4  ");
    Serial.println("    For Waveshare RP2350-Zero      ");
    Serial.println("=====================================");
    
    // Initialize hardware
    initializeHardware();
    Serial.println("[OK] Hardware initialized");
    #ifdef HAS_RP2040_HARDWARE
    Serial.println("     - I2C internal pull-ups enabled");
    #else
    Serial.println("     - Using Arduino pull-ups");
    #endif
    
    // Initialize buffer
    initializeBuffer();
    Serial.println("[OK] Buffer initialized (4KB)");
    
    // Initialize UART for GNSS
    #ifdef HAS_RP2040_HARDWARE
    Serial1.setRX(UART_RX_PIN);
    Serial1.setTX(UART_TX_PIN);
    #endif
    Serial1.begin(UART_BAUD_RATE);
    Serial.println("[OK] UART initialized");
    Serial.print("     - RX: GPIO"); Serial.println(UART_RX_PIN);
    Serial.print("     - TX: GPIO"); Serial.println(UART_TX_PIN);
    Serial.print("     - Baud: "); Serial.println(UART_BAUD_RATE);
    
    // Initialize I2C slave
    #ifdef HAS_RP2040_HARDWARE
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    #endif
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    
    Serial.println("[OK] I2C slave initialized");
    Serial.print("     - Address: 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
    Serial.print("     - SDA: GPIO"); Serial.println(I2C_SDA_PIN);
    Serial.print("     - SCL: GPIO"); Serial.println(I2C_SCL_PIN);
    Serial.print("     - Packet size: "); Serial.print(I2C_PACKET_SIZE); Serial.println(" bytes");
    
    Serial.println("\n[READY] Waiting for NMEA data...\n");
}

// Core 0 main loop - UART processing
void loop() {
    // Process all available UART data
    while (Serial1.available()) {
        char c = Serial1.read();
        processUARTChar(c);
    }
    
    // Update status LED
    updateStatusIndicator();
    
    // Periodic status report
    static unsigned long lastStatusReport = 0;
    if (millis() - lastStatusReport > 10000) {
        lastStatusReport = millis();
        
        // Print status
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
        
        // Clear error flags
        nmeaData.status.checksumError = 0;
        nmeaData.status.uartError = 0;
        nmeaData.status.bufferOverflow = 0;
    }
}

// Core 1 setup (optional - for future dual-core features)
void setup1() {
    // Core 1 can be used for additional processing
    // Currently not used
    delay(100);
}

// Core 1 loop
void loop1() {
    // Currently not used, just yield CPU time
    delay(1);
}

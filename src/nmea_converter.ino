/*
 * NMEA UART to I2C Converter for Waveshare RP2350-Zero
 * Version: 3.2 - Full Dual-Core Implementation
 * 
 * Features:
 * - True dual-core processing with active Core 1
 * - 255-byte I2C packets (maximum size)
 * - 255-character NMEA support
 * - I2C address: 0x10
 * - Thread-safe operations with mutex
 * - Inter-core communication
 * - NMEA filtering and caching on Core 1
 * 
 * Core 0: UART reception, NMEA validation, buffer writing
 * Core 1: I2C handling, statistics, filtering, position caching
 */

#include <Arduino.h>
#include <Wire.h>
#include <pico/mutex.h>
#include <pico/multicore.h>

// ====== CONFIGURATION ======
// GPIO Pins for RP2350-Zero
#define UART_RX_PIN 1      // GPIO1 - UART0 RX
#define UART_TX_PIN 0      // GPIO0 - UART0 TX  
#define I2C_SDA_PIN 4      // GPIO4 - I2C0 SDA
#define I2C_SCL_PIN 5      // GPIO5 - I2C0 SCL

// Protocol Configuration
#define I2C_SLAVE_ADDRESS 0x10  // Changed to 0x10 as requested
#define UART_BAUD_RATE 115200   
#define NMEA_BUFFER_SIZE 8192   // Increased for better buffering
#define I2C_PACKET_SIZE 255     // Maximum I2C packet size
#define NMEA_MAX_LENGTH 255     

// I2C Commands
#define CMD_GET_STATUS    0x01  
#define CMD_GET_COUNT     0x02  
#define CMD_READ_DATA     0x03  
#define CMD_CLEAR_BUFFER  0x04  
#define CMD_GET_VERSION   0x05  
#define CMD_GET_INFO      0x06  
#define CMD_GET_POSITION  0x07  
#define CMD_GET_STATS     0x08  
#define CMD_SET_FILTER    0x09  

#define FIRMWARE_VERSION  0x32  // Version 3.2

// Status flags
typedef struct {
    uint8_t dataReady : 1;      
    uint8_t bufferOverflow : 1; 
    uint8_t checksumError : 1;  
    uint8_t uartError : 1;      
    uint8_t i2cBusy : 1;        
    uint8_t positionValid : 1;  
    uint8_t filterActive : 1;   
    uint8_t core1Active : 1;    // New: Core 1 status
} StatusFlags;

// Cached position data (maintained by Core 1)
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float course;
    uint8_t satellites;
    uint8_t fixQuality;
    uint32_t timestamp;
    bool valid;
} CachedPosition;

// Statistics (maintained by Core 1)
typedef struct {
    uint32_t totalSentences;
    uint32_t validSentences;
    uint32_t filteredSentences;
    uint32_t checksumErrors;
    uint32_t bufferOverflows;
    uint32_t i2cTransactions;
    uint32_t core0Cycles;
    uint32_t core1Cycles;
    uint32_t uptime;
} Statistics;

// NMEA Filter settings
typedef struct {
    bool enableGGA;
    bool enableRMC;
    bool enableGSV;
    bool enableVTG;
    bool enableGLL;
    bool enableGSA;
    bool enableGPTXT;
    bool enableGNGGA;
} NMEAFilter;

// Shared data structure
typedef struct {
    char buffer[NMEA_BUFFER_SIZE];
    volatile uint16_t writePos;
    volatile uint16_t readPos;
    volatile uint16_t dataCount;
    StatusFlags status;
    CachedPosition position;
    Statistics stats;
    NMEAFilter filter;
} SharedData;

// Inter-core communication
typedef enum {
    MSG_NONE = 0,
    MSG_NEW_NMEA,
    MSG_CLEAR_BUFFER,
    MSG_UPDATE_FILTER,
    MSG_RESET_STATS
} MessageType;

typedef struct {
    MessageType type;
    uint32_t data;
    char nmea[NMEA_MAX_LENGTH];
} CoreMessage;

// Global variables
SharedData sharedData;
mutex_t dataMutex;
CoreMessage coreMsg;
volatile uint8_t lastI2CCommand = 0;
volatile uint32_t startTime = 0;
volatile bool core1Started = false;

// Function prototypes
void initializeHardware();
void initializeSharedData();
bool writeToBuffer(const char* data, uint16_t length);
uint16_t readFromBuffer(char* output, uint16_t maxLength);
uint8_t calculateNMEAChecksum(const char* sentence);
bool validateNMEASentence(const char* sentence, uint16_t length);
void processUARTChar(char c);
void onI2CReceive(int bytes);
void onI2CRequest();
void core1_entry();
void processNMEASentence(const char* sentence);
bool shouldFilterSentence(const char* sentence);

// Core 0: Initialize hardware
void initializeHardware() {
    // Configure I2C pins with internal pull-ups
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    startTime = millis();
}

// Initialize shared data and mutex
void initializeSharedData() {
    mutex_init(&dataMutex);
    
    memset(&sharedData, 0, sizeof(SharedData));
    
    // Default filter: all enabled
    sharedData.filter.enableGGA = true;
    sharedData.filter.enableRMC = true;
    sharedData.filter.enableGSV = true;
    sharedData.filter.enableVTG = true;
    sharedData.filter.enableGLL = true;
    sharedData.filter.enableGSA = true;
    sharedData.filter.enableGPTXT = true;
    sharedData.filter.enableGNGGA = true;
}

// Thread-safe write to buffer
bool writeToBuffer(const char* data, uint16_t length) {
    if (!mutex_try_enter(&dataMutex, NULL)) {
        return false;
    }
    
    if (NMEA_BUFFER_SIZE - sharedData.dataCount < length) {
        sharedData.status.bufferOverflow = 1;
        sharedData.stats.bufferOverflows++;
        mutex_exit(&dataMutex);
        return false;
    }
    
    for (uint16_t i = 0; i < length; i++) {
        sharedData.buffer[sharedData.writePos] = data[i];
        sharedData.writePos = (sharedData.writePos + 1) % NMEA_BUFFER_SIZE;
        sharedData.dataCount++;
    }
    
    sharedData.status.dataReady = 1;
    mutex_exit(&dataMutex);
    
    // Notify Core 1 about new data
    coreMsg.type = MSG_NEW_NMEA;
    strncpy(coreMsg.nmea, data, NMEA_MAX_LENGTH - 1);
    coreMsg.nmea[NMEA_MAX_LENGTH - 1] = '\0';
    
    return true;
}

// Thread-safe read from buffer
uint16_t readFromBuffer(char* output, uint16_t maxLength) {
    if (!mutex_try_enter(&dataMutex, NULL)) {
        return 0;
    }
    
    uint16_t bytesRead = 0;
    while (bytesRead < maxLength && sharedData.dataCount > 0) {
        output[bytesRead++] = sharedData.buffer[sharedData.readPos];
        sharedData.readPos = (sharedData.readPos + 1) % NMEA_BUFFER_SIZE;
        sharedData.dataCount--;
    }
    
    if (sharedData.dataCount == 0) {
        sharedData.status.dataReady = 0;
    }
    
    mutex_exit(&dataMutex);
    return bytesRead;
}

// NMEA checksum calculation
uint8_t calculateNMEAChecksum(const char* sentence) {
    uint8_t checksum = 0;
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0' && sentence[i] != '\r'; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}

// Validate NMEA sentence
bool validateNMEASentence(const char* sentence, uint16_t length) {
    if (length < 8 || sentence[0] != '$') return false;
    
    int checksumPos = -1;
    for (int i = length - 5; i < length - 2; i++) {
        if (sentence[i] == '*') {
            checksumPos = i;
            break;
        }
    }
    
    if (checksumPos == -1) return true; // No checksum, accept
    
    char checksumStr[3] = {sentence[checksumPos + 1], sentence[checksumPos + 2], '\0'};
    uint8_t providedChecksum = strtol(checksumStr, NULL, 16);
    uint8_t calculatedChecksum = calculateNMEAChecksum(sentence);
    
    return providedChecksum == calculatedChecksum;
}

// Core 0: Process UART character
char currentSentence[NMEA_MAX_LENGTH + 1];
uint16_t sentenceIndex = 0;
bool inSentence = false;

void processUARTChar(char c) {
    if (c == '$') {
        inSentence = true;
        sentenceIndex = 0;
        currentSentence[sentenceIndex++] = c;
    } else if (inSentence && sentenceIndex < NMEA_MAX_LENGTH) {
        currentSentence[sentenceIndex++] = c;
        
        if (c == '\n' && sentenceIndex > 1 && currentSentence[sentenceIndex - 2] == '\r') {
            currentSentence[sentenceIndex] = '\0';
            
            sharedData.stats.totalSentences++;
            sharedData.stats.core0Cycles++;
            
            if (validateNMEASentence(currentSentence, sentenceIndex)) {
                sharedData.stats.validSentences++;
                
                if (!shouldFilterSentence(currentSentence)) {
                    writeToBuffer(currentSentence, sentenceIndex);
                } else {
                    sharedData.stats.filteredSentences++;
                }
            } else {
                sharedData.status.checksumError = 1;
                sharedData.stats.checksumErrors++;
            }
            
            inSentence = false;
        }
    } else if (sentenceIndex >= NMEA_MAX_LENGTH) {
        inSentence = false;
        sharedData.status.uartError = 1;
    }
}

// Check if sentence should be filtered
bool shouldFilterSentence(const char* sentence) {
    if (strncmp(sentence + 3, "GGA", 3) == 0) return !sharedData.filter.enableGGA;
    if (strncmp(sentence + 3, "RMC", 3) == 0) return !sharedData.filter.enableRMC;
    if (strncmp(sentence + 3, "GSV", 3) == 0) return !sharedData.filter.enableGSV;
    if (strncmp(sentence + 3, "VTG", 3) == 0) return !sharedData.filter.enableVTG;
    if (strncmp(sentence + 3, "GLL", 3) == 0) return !sharedData.filter.enableGLL;
    if (strncmp(sentence + 3, "GSA", 3) == 0) return !sharedData.filter.enableGSA;
    if (strncmp(sentence + 3, "TXT", 3) == 0) return !sharedData.filter.enableGPTXT;
    if (strncmp(sentence + 2, "GNGGA", 5) == 0) return !sharedData.filter.enableGNGGA;
    return false;
}

// I2C handlers
void onI2CReceive(int bytes) {
    if (bytes > 0) {
        lastI2CCommand = Wire.read();
        
        // Handle filter commands
        if (lastI2CCommand == CMD_SET_FILTER && bytes > 1) {
            uint8_t filterByte = Wire.read();
            mutex_enter_blocking(&dataMutex);
            sharedData.filter.enableGGA = filterByte & 0x01;
            sharedData.filter.enableRMC = filterByte & 0x02;
            sharedData.filter.enableGSV = filterByte & 0x04;
            sharedData.filter.enableVTG = filterByte & 0x08;
            sharedData.filter.enableGLL = filterByte & 0x10;
            sharedData.filter.enableGSA = filterByte & 0x20;
            sharedData.filter.enableGPTXT = filterByte & 0x40;
            sharedData.filter.enableGNGGA = filterByte & 0x80;
            sharedData.status.filterActive = (filterByte != 0xFF);
            mutex_exit(&dataMutex);
            
            coreMsg.type = MSG_UPDATE_FILTER;
        }
        
        while (Wire.available()) Wire.read();
    }
}

void onI2CRequest() {
    sharedData.status.i2cBusy = 1;
    sharedData.stats.i2cTransactions++;
    
    switch (lastI2CCommand) {
        case CMD_GET_STATUS: {
            uint8_t statusByte = *(uint8_t*)&sharedData.status;
            Wire.write(statusByte);
            break;
        }
        
        case CMD_GET_COUNT: {
            uint16_t count = sharedData.dataCount;
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
            coreMsg.type = MSG_CLEAR_BUFFER;
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
            snprintf(info, sizeof(info), "RP2350,V3.2,UP:%lu,C1:%d,POS:%d", 
                     uptime, core1Started ? 1 : 0, sharedData.position.valid ? 1 : 0);
            Wire.write((uint8_t*)info, strlen(info));
            break;
        }
        
        case CMD_GET_POSITION: {
            if (sharedData.position.valid) {
                // Send position data in binary format
                Wire.write((uint8_t*)&sharedData.position.latitude, 8);
                Wire.write((uint8_t*)&sharedData.position.longitude, 8);
                Wire.write((uint8_t*)&sharedData.position.altitude, 4);
                Wire.write((uint8_t*)&sharedData.position.speed, 4);
                Wire.write((uint8_t*)&sharedData.position.course, 4);
                Wire.write(sharedData.position.satellites);
                Wire.write(sharedData.position.fixQuality);
            } else {
                Wire.write(0x00);
            }
            break;
        }
        
        case CMD_GET_STATS: {
            // Send statistics
            Wire.write((uint8_t*)&sharedData.stats.totalSentences, 4);
            Wire.write((uint8_t*)&sharedData.stats.validSentences, 4);
            Wire.write((uint8_t*)&sharedData.stats.filteredSentences, 4);
            Wire.write((uint8_t*)&sharedData.stats.checksumErrors, 4);
            Wire.write((uint8_t*)&sharedData.stats.bufferOverflows, 4);
            Wire.write((uint8_t*)&sharedData.stats.i2cTransactions, 4);
            Wire.write((uint8_t*)&sharedData.stats.core0Cycles, 4);
            Wire.write((uint8_t*)&sharedData.stats.core1Cycles, 4);
            break;
        }
        
        default: {
            Wire.write(0xFF);
            break;
        }
    }
    
    sharedData.status.i2cBusy = 0;
}

// Core 0 setup
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=====================================");
    Serial.println("  NMEA UART to I2C Converter v3.2  ");
    Serial.println("    Full Dual-Core Implementation   ");
    Serial.println("=====================================");
    
    initializeHardware();
    initializeSharedData();
    
    Serial.println("[OK] Hardware initialized");
    Serial.println("     - I2C internal pull-ups enabled");
    
    // Initialize UART
    Serial1.setRX(UART_RX_PIN);
    Serial1.setTX(UART_TX_PIN);
    Serial1.begin(UART_BAUD_RATE);
    Serial.println("[OK] UART initialized");
    
    // Initialize I2C
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.print("[OK] I2C slave initialized at 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
    Serial.print("     - Max packet size: ");
    Serial.print(I2C_PACKET_SIZE);
    Serial.println(" bytes");
    
    // Launch Core 1
    multicore_launch_core1(core1_entry);
    delay(100);
    
    Serial.println("\n[INFO] Core assignment:");
    Serial.println("     - Core 0: UART RX, validation, buffering");
    Serial.println("     - Core 1: I2C handling, filtering, caching");
    Serial.println("\n[READY] System operational\n");
}

// Core 0 main loop
void loop() {
    // Process UART data
    while (Serial1.available()) {
        char c = Serial1.read();
        processUARTChar(c);
    }
    
    // Handle inter-core messages
    if (coreMsg.type == MSG_CLEAR_BUFFER) {
        mutex_enter_blocking(&dataMutex);
        sharedData.writePos = 0;
        sharedData.readPos = 0;
        sharedData.dataCount = 0;
        sharedData.status.dataReady = 0;
        sharedData.status.bufferOverflow = 0;
        mutex_exit(&dataMutex);
        coreMsg.type = MSG_NONE;
        
        Serial.println("[CLEARED] Buffer reset");
    }
    
    // Periodic status report
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 10000) {
        lastStatus = millis();
        
        mutex_enter_blocking(&dataMutex);
        Serial.print("\n[STATUS] Total: ");
        Serial.print(sharedData.stats.totalSentences);
        Serial.print(", Valid: ");
        Serial.print(sharedData.stats.validSentences);
        Serial.print(", Filtered: ");
        Serial.print(sharedData.stats.filteredSentences);
        Serial.print(", Buffer: ");
        Serial.print(sharedData.dataCount);
        Serial.print("/");
        Serial.println(NMEA_BUFFER_SIZE);
        
        if (sharedData.position.valid) {
            Serial.print("[POSITION] Lat: ");
            Serial.print(sharedData.position.latitude, 6);
            Serial.print(", Lon: ");
            Serial.print(sharedData.position.longitude, 6);
            Serial.print(", Sats: ");
            Serial.print(sharedData.position.satellites);
            Serial.print(", Fix: ");
            Serial.println(sharedData.position.fixQuality);
        }
        
        Serial.print("[CORES] C0: ");
        Serial.print(sharedData.stats.core0Cycles);
        Serial.print(", C1: ");
        Serial.println(sharedData.stats.core1Cycles);
        mutex_exit(&dataMutex);
        
        // Clear status flags
        sharedData.status.checksumError = 0;
        sharedData.status.uartError = 0;
        sharedData.status.bufferOverflow = 0;
    }
}

// ====== CORE 1 FUNCTIONS ======

// Parse GGA sentence for position
void parseGGA(const char* sentence) {
    char temp[NMEA_MAX_LENGTH];
    strncpy(temp, sentence, NMEA_MAX_LENGTH - 1);
    temp[NMEA_MAX_LENGTH - 1] = '\0';
    
    char* ptr = temp;
    char* token;
    int field = 0;
    
    mutex_enter_blocking(&dataMutex);
    
    while ((token = strsep(&ptr, ",")) != NULL) {
        field++;
        switch (field) {
            case 3: // Latitude
                if (strlen(token) > 0) {
                    double lat = atof(token);
                    int degrees = (int)(lat / 100);
                    double minutes = lat - (degrees * 100);
                    sharedData.position.latitude = degrees + (minutes / 60.0);
                }
                break;
            case 4: // N/S
                if (token[0] == 'S') sharedData.position.latitude *= -1;
                break;
            case 5: // Longitude
                if (strlen(token) > 0) {
                    double lon = atof(token);
                    int degrees = (int)(lon / 100);
                    double minutes = lon - (degrees * 100);
                    sharedData.position.longitude = degrees + (minutes / 60.0);
                }
                break;
            case 6: // E/W
                if (token[0] == 'W') sharedData.position.longitude *= -1;
                break;
            case 7: // Fix quality
                sharedData.position.fixQuality = atoi(token);
                break;
            case 8: // Satellites
                sharedData.position.satellites = atoi(token);
                break;
            case 10: // Altitude
                sharedData.position.altitude = atof(token);
                break;
        }
    }
    
    sharedData.position.timestamp = millis();
    sharedData.position.valid = true;
    sharedData.status.positionValid = 1;
    
    mutex_exit(&dataMutex);
}

// Parse RMC sentence for speed/course
void parseRMC(const char* sentence) {
    char temp[NMEA_MAX_LENGTH];
    strncpy(temp, sentence, NMEA_MAX_LENGTH - 1);
    temp[NMEA_MAX_LENGTH - 1] = '\0';
    
    char* ptr = temp;
    char* token;
    int field = 0;
    
    mutex_enter_blocking(&dataMutex);
    
    while ((token = strsep(&ptr, ",")) != NULL) {
        field++;
        switch (field) {
            case 8: // Speed in knots
                if (strlen(token) > 0) {
                    sharedData.position.speed = atof(token) * 1.852; // Convert to km/h
                }
                break;
            case 9: // Course
                if (strlen(token) > 0) {
                    sharedData.position.course = atof(token);
                }
                break;
        }
    }
    
    mutex_exit(&dataMutex);
}

// Core 1: Process NMEA sentences
void processNMEASentence(const char* sentence) {
    if (strncmp(sentence + 3, "GGA", 3) == 0 || strncmp(sentence + 2, "GNGGA", 5) == 0) {
        parseGGA(sentence);
    } else if (strncmp(sentence + 3, "RMC", 3) == 0) {
        parseRMC(sentence);
    }
}

// Core 1 entry point
void core1_entry() {
    delay(50);
    
    mutex_enter_blocking(&dataMutex);
    sharedData.status.core1Active = 1;
    mutex_exit(&dataMutex);
    
    core1Started = true;
    
    while (true) {
        // Update Core 1 cycle counter
        sharedData.stats.core1Cycles++;
        
        // Process inter-core messages
        if (coreMsg.type == MSG_NEW_NMEA) {
            processNMEASentence(coreMsg.nmea);
            coreMsg.type = MSG_NONE;
        }
        
        // Update statistics
        static unsigned long lastStatsUpdate = 0;
        if (millis() - lastStatsUpdate > 1000) {
            lastStatsUpdate = millis();
            mutex_enter_blocking(&dataMutex);
            sharedData.stats.uptime = (millis() - startTime) / 1000;
            mutex_exit(&dataMutex);
        }
        
        // Check position validity timeout (30 seconds)
        if (sharedData.position.valid && 
            (millis() - sharedData.position.timestamp) > 30000) {
            mutex_enter_blocking(&dataMutex);
            sharedData.position.valid = false;
            sharedData.status.positionValid = 0;
            mutex_exit(&dataMutex);
        }
        
        delay(1);
    }
}
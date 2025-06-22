/*
 * NMEA Parser Test for PortaPack H4M
 * Simulates PortaPack GPS app behavior
 * 
 * Tests:
 * 1. I2C communication with converter
 * 2. NMEA sentence parsing
 * 3. GPS data extraction
 * 
 * Note: PortaPack expects GPS on 0x10, but our converter is on 0x42
 * You'll need to change address in PortaPack app or converter
 */

#include <Wire.h>

// Configuration
#define CONVERTER_ADDRESS 0x42  // Our converter address
#define PORTAPACK_GPS_ADDRESS 0x10  // PortaPack expects this

// I2C Commands
#define CMD_GET_STATUS    0x01
#define CMD_GET_COUNT     0x02
#define CMD_READ_DATA     0x03
#define CMD_CLEAR_BUFFER  0x04

// GPS Data Structure (similar to PortaPack)
struct GPSData {
  float latitude;
  float longitude;
  float altitude;
  float speed;
  int satellites;
  int hours;
  int minutes;
  int seconds;
  bool fixValid;
  
  // Satellite counts by system
  int gpsCount;
  int glonassCount;
  int galileoCount;
  int beidouCount;
};

GPSData gpsData = {0};
char nmeaLine[100];
int nmeaIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  
  Serial.println("\n=====================================");
  Serial.println("  NMEA Parser Test (PortaPack sim) ");
  Serial.println("=====================================");
  
  // Initialize I2C
  Wire.begin();
  
  Serial.print("\nChecking for converter at 0x");
  Serial.print(CONVERTER_ADDRESS, HEX);
  Serial.print("... ");
  
  Wire.beginTransmission(CONVERTER_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("FOUND!");
  } else {
    Serial.println("NOT FOUND!");
  }
  
  Serial.println("\nIMPORTANT: PortaPack expects GPS at 0x10");
  Serial.println("Current converter is at 0x42");
  Serial.println("Change #define I2C_SLAVE_ADDRESS in converter\n");
  
  Serial.println("[READY] Reading NMEA data...\n");
}

void loop() {
  // Read data from converter
  uint16_t available = getByteCount();
  
  if (available > 0) {
    // Read in chunks
    while (available > 0) {
      int toRead = min(32, available);
      char buffer[33];
      int bytesRead = readData(buffer, toRead);
      
      // Process each character
      for (int i = 0; i < bytesRead; i++) {
        processNMEAChar(buffer[i]);
      }
      
      available -= bytesRead;
    }
  }
  
  // Display GPS data every 2 seconds
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 2000) {
    lastDisplay = millis();
    displayGPSData();
  }
  
  delay(100);
}

// Process NMEA character by character
void processNMEAChar(char c) {
  if (c == '$') {
    // Start of new sentence
    nmeaIndex = 0;
    nmeaLine[nmeaIndex++] = c;
  } else if (nmeaIndex < 99) {
    nmeaLine[nmeaIndex++] = c;
    
    // Check for end of line
    if (c == '\n') {
      nmeaLine[nmeaIndex] = '\0';
      parseNMEASentence(nmeaLine);
      nmeaIndex = 0;
    }
  }
}

// Parse NMEA sentence
void parseNMEASentence(char* sentence) {
  // Show raw sentence
  Serial.print("[NMEA] ");
  Serial.print(sentence);
  
  // Parse different sentence types
  if (strstr(sentence, "$GPGGA") || strstr(sentence, "$GNGGA")) {
    parseGGA(sentence);
  } else if (strstr(sentence, "$GPRMC") || strstr(sentence, "$GNRMC")) {
    parseRMC(sentence);
  } else if (strstr(sentence, "$GPGSV") || strstr(sentence, "$GLGSV") || 
             strstr(sentence, "$GAGSV") || strstr(sentence, "$GBGSV")) {
    parseGSV(sentence);
  }
}

// Parse GGA sentence (position)
void parseGGA(char* sentence) {
  char* token = strtok(sentence, ",");
  int field = 0;
  
  while (token != NULL) {
    switch (field) {
      case 1: // Time
        if (strlen(token) >= 6) {
          gpsData.hours = (token[0] - '0') * 10 + (token[1] - '0');
          gpsData.minutes = (token[2] - '0') * 10 + (token[3] - '0');
          gpsData.seconds = (token[4] - '0') * 10 + (token[5] - '0');
        }
        break;
        
      case 2: // Latitude
        if (strlen(token) > 0) {
          float lat = atof(token);
          int degrees = (int)(lat / 100);
          float minutes = lat - (degrees * 100);
          gpsData.latitude = degrees + (minutes / 60.0);
        }
        break;
        
      case 3: // N/S
        if (token[0] == 'S') gpsData.latitude = -gpsData.latitude;
        break;
        
      case 4: // Longitude  
        if (strlen(token) > 0) {
          float lon = atof(token);
          int degrees = (int)(lon / 100);
          float minutes = lon - (degrees * 100);
          gpsData.longitude = degrees + (minutes / 60.0);
        }
        break;
        
      case 5: // E/W
        if (token[0] == 'W') gpsData.longitude = -gpsData.longitude;
        break;
        
      case 6: // Fix quality
        gpsData.fixValid = (atoi(token) > 0);
        break;
        
      case 7: // Satellites
        gpsData.satellites = atoi(token);
        break;
        
      case 9: // Altitude
        gpsData.altitude = atof(token);
        break;
    }
    
    field++;
    token = strtok(NULL, ",");
  }
}

// Parse RMC sentence (recommended minimum)
void parseRMC(char* sentence) {
  char* token = strtok(sentence, ",");
  int field = 0;
  
  while (token != NULL) {
    switch (field) {
      case 2: // Status
        gpsData.fixValid = (token[0] == 'A');
        break;
        
      case 7: // Speed in knots
        gpsData.speed = atof(token) * 1.852; // Convert to km/h
        break;
    }
    
    field++;
    token = strtok(NULL, ",");
  }
}

// Parse GSV sentence (satellites in view)
void parseGSV(char* sentence) {
  // Identify satellite system
  if (strstr(sentence, "$GPGSV")) {
    // GPS satellites (PRN 1-32)
    // Count based on sentence info
  } else if (strstr(sentence, "$GLGSV")) {
    // GLONASS satellites (PRN 65-96)
  } else if (strstr(sentence, "$GAGSV")) {
    // Galileo satellites
  } else if (strstr(sentence, "$GBGSV")) {
    // BeiDou satellites
  }
}

// Display GPS data like PortaPack
void displayGPSData() {
  Serial.println("\n===== GPS STATUS (PortaPack Format) =====");
  
  Serial.print("Fix: ");
  Serial.println(gpsData.fixValid ? "VALID" : "NO FIX");
  
  Serial.print("UTC Time: ");
  if (gpsData.hours < 10) Serial.print("0");
  Serial.print(gpsData.hours);
  Serial.print(":");
  if (gpsData.minutes < 10) Serial.print("0");
  Serial.print(gpsData.minutes);
  Serial.print(":");
  if (gpsData.seconds < 10) Serial.print("0");
  Serial.println(gpsData.seconds);
  
  Serial.print("Latitude: ");
  Serial.print(gpsData.latitude, 6);
  Serial.println("°");
  
  Serial.print("Longitude: ");
  Serial.print(gpsData.longitude, 6);
  Serial.println("°");
  
  Serial.print("Altitude: ");
  Serial.print(gpsData.altitude, 1);
  Serial.println(" m");
  
  Serial.print("Speed: ");
  Serial.print(gpsData.speed, 1);
  Serial.println(" km/h");
  
  Serial.print("Satellites: ");
  Serial.print(gpsData.satellites);
  Serial.println(" in use");
  
  Serial.println("=========================================\n");
}

// I2C Functions
uint16_t getByteCount() {
  Wire.beginTransmission(CONVERTER_ADDRESS);
  Wire.write(CMD_GET_COUNT);
  Wire.endTransmission();
  
  Wire.requestFrom(CONVERTER_ADDRESS, 2);
  if (Wire.available() >= 2) {
    uint16_t count = Wire.read();
    count |= (Wire.read() << 8);
    return count;
  }
  return 0;
}

int readData(char* buffer, int maxBytes) {
  Wire.beginTransmission(CONVERTER_ADDRESS);
  Wire.write(CMD_READ_DATA);
  Wire.endTransmission();
  
  int requested = min(32, maxBytes);
  Wire.requestFrom(CONVERTER_ADDRESS, requested);
  
  int bytesRead = 0;
  while (Wire.available() && bytesRead < maxBytes) {
    buffer[bytesRead++] = Wire.read();
  }
  
  return bytesRead;
}
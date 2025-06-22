/*
 * NMEA Test Generator
 * Generates synthetic NMEA data for testing UART to I2C converter
 * Compatible with: RP2040-Tiny, ESP32-C3-Mini, Arduino, etc.
 * 
 * Connections:
 * Generator TX (GPIO0) -> Converter RX (GPIO1)
 * Generator GND -> Converter GND
 * 
 * Author: NMEA Converter Test Suite
 * License: MIT
 */

// Configuration
#define NMEA_SEND_INTERVAL 1000  // Send NMEA sentence every 1 second
#define USE_REAL_COORDINATES true // Use real coordinates or test pattern

// Pin configuration - adjust for your board
#ifdef ESP32
  #define TX_PIN 1   // ESP32-C3 default TX
  #define LED_PIN 2  // ESP32-C3 onboard LED
#elif defined(ARDUINO_ARCH_RP2040)
  #define TX_PIN 0   // RP2040 default TX
  #define LED_PIN 25 // RP2040 onboard LED (if available)
#else
  #define TX_PIN 1   // Default TX
  #define LED_PIN LED_BUILTIN
#endif

// Global variables
float latitude = 55.7558;   // Moscow latitude
float longitude = 37.6173;  // Moscow longitude
float altitude = 156.0;     // meters
int satellites = 8;
float hdop = 1.2;
unsigned long lastSentTime = 0;
int sentenceCounter = 0;

// Function prototypes
String generateGPGGA();
String generateGPRMC();
String generateGPGSV();
String generateGPVTG();
String calculateNMEAChecksum(String sentence);
void sendNMEASentence(String sentence);

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize UART for NMEA output
  #ifdef ESP32
    Serial1.begin(115200, SERIAL_8N1, -1, TX_PIN); // ESP32 allows pin remapping
  #else
    Serial1.begin(115200);
  #endif
  
  // Configure LED
  pinMode(LED_PIN, OUTPUT);
  
  // Startup message
  Serial.println("=================================");
  Serial.println("    NMEA Test Generator v1.0    ");
  Serial.println("=================================");
  Serial.print("Sending NMEA on TX pin: GPIO");
  Serial.println(TX_PIN);
  Serial.print("Interval: ");
  Serial.print(NMEA_SEND_INTERVAL);
  Serial.println(" ms");
  Serial.println("\nStarting NMEA generation...\n");
  
  // Blink LED 3 times to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send NMEA sentences at regular intervals
  if (currentTime - lastSentTime >= NMEA_SEND_INTERVAL) {
    lastSentTime = currentTime;
    
    // Toggle LED to indicate transmission
    digitalWrite(LED_PIN, HIGH);
    
    // Generate and send different NMEA sentences
    switch (sentenceCounter % 4) {
      case 0:
        sendNMEASentence(generateGPGGA());
        break;
      case 1:
        sendNMEASentence(generateGPRMC());
        break;
      case 2:
        sendNMEASentence(generateGPGSV());
        break;
      case 3:
        sendNMEASentence(generateGPVTG());
        break;
    }
    
    sentenceCounter++;
    
    // Simulate movement (optional)
    if (USE_REAL_COORDINATES) {
      latitude += 0.00001;   // Small movement north
      longitude += 0.00001;  // Small movement east
      altitude += 0.1;       // Small altitude change
    }
    
    // Turn off LED
    digitalWrite(LED_PIN, LOW);
  }
}

// Generate GPGGA sentence (GPS fix data)
String generateGPGGA() {
  // Get current time
  int hours = (millis() / 3600000) % 24;
  int minutes = (millis() / 60000) % 60;
  int seconds = (millis() / 1000) % 60;
  
  String sentence = "$GPGGA,";
  
  // Time: HHMMSS.SS
  sentence += String(hours, DEC).length() == 1 ? "0" + String(hours) : String(hours);
  sentence += String(minutes, DEC).length() == 1 ? "0" + String(minutes) : String(minutes);
  sentence += String(seconds, DEC).length() == 1 ? "0" + String(seconds) : String(seconds);
  sentence += ".00,";
  
  // Latitude: DDMM.MMMM,N/S
  int latDeg = (int)latitude;
  float latMin = (latitude - latDeg) * 60;
  sentence += String(latDeg, DEC).length() == 1 ? "0" + String(latDeg) : String(latDeg);
  sentence += String(latMin, 4);
  sentence += ",N,";
  
  // Longitude: DDDMM.MMMM,E/W
  int lonDeg = (int)longitude;
  float lonMin = (longitude - lonDeg) * 60;
  sentence += String(lonDeg, DEC).length() == 2 ? "0" + String(lonDeg) : String(lonDeg);
  sentence += String(lonMin, 4);
  sentence += ",E,";
  
  // Fix quality: 1 = GPS fix
  sentence += "1,";
  
  // Number of satellites
  sentence += String(satellites) + ",";
  
  // HDOP
  sentence += String(hdop, 1) + ",";
  
  // Altitude
  sentence += String(altitude, 1) + ",M,";
  
  // Height of geoid
  sentence += "46.9,M,,";
  
  // Add checksum
  sentence += "*" + calculateNMEAChecksum(sentence);
  
  return sentence;
}

// Generate GPRMC sentence (Recommended minimum data)
String generateGPRMC() {
  // Get current time
  int hours = (millis() / 3600000) % 24;
  int minutes = (millis() / 60000) % 60;
  int seconds = (millis() / 1000) % 60;
  
  String sentence = "$GPRMC,";
  
  // Time
  sentence += String(hours, DEC).length() == 1 ? "0" + String(hours) : String(hours);
  sentence += String(minutes, DEC).length() == 1 ? "0" + String(minutes) : String(minutes);
  sentence += String(seconds, DEC).length() == 1 ? "0" + String(seconds) : String(seconds);
  sentence += ".00,";
  
  // Status: A = active
  sentence += "A,";
  
  // Latitude
  int latDeg = (int)latitude;
  float latMin = (latitude - latDeg) * 60;
  sentence += String(latDeg, DEC).length() == 1 ? "0" + String(latDeg) : String(latDeg);
  sentence += String(latMin, 4);
  sentence += ",N,";
  
  // Longitude
  int lonDeg = (int)longitude;
  float lonMin = (longitude - lonDeg) * 60;
  sentence += String(lonDeg, DEC).length() == 2 ? "0" + String(lonDeg) : String(lonDeg);
  sentence += String(lonMin, 4);
  sentence += ",E,";
  
  // Speed in knots
  sentence += "0.5,";
  
  // Track angle
  sentence += "45.0,";
  
  // Date: DDMMYY
  sentence += "220625,";
  
  // Magnetic variation
  sentence += ",,";
  
  // Add checksum
  sentence += "*" + calculateNMEAChecksum(sentence);
  
  return sentence;
}

// Generate GPGSV sentence (Satellites in view)
String generateGPGSV() {
  String sentence = "$GPGSV,3,1,";
  
  // Total satellites in view
  sentence += String(satellites) + ",";
  
  // Satellite 1: PRN, elevation, azimuth, SNR
  sentence += "01,45,045,50,";
  
  // Satellite 2
  sentence += "02,30,120,45,";
  
  // Satellite 3
  sentence += "03,60,180,48,";
  
  // Satellite 4
  sentence += "04,25,270,40";
  
  // Add checksum
  sentence += "*" + calculateNMEAChecksum(sentence);
  
  return sentence;
}

// Generate GPVTG sentence (Track made good and ground speed)
String generateGPVTG() {
  String sentence = "$GPVTG,";
  
  // Track made good (degrees true)
  sentence += "45.0,T,";
  
  // Track made good (degrees magnetic)
  sentence += ",M,";
  
  // Speed in knots
  sentence += "0.5,N,";
  
  // Speed in km/h
  sentence += "0.9,K,";
  
  // Mode indicator: A = Autonomous
  sentence += "A";
  
  // Add checksum
  sentence += "*" + calculateNMEAChecksum(sentence);
  
  return sentence;
}

// Calculate NMEA checksum
String calculateNMEAChecksum(String sentence) {
  byte checksum = 0;
  
  // Start after '$' and calculate until '*' or end
  for (int i = 1; i < sentence.length(); i++) {
    if (sentence[i] == '*') break;
    checksum ^= sentence[i];
  }
  
  // Convert to hex string
  String checksumStr = String(checksum, HEX);
  checksumStr.toUpperCase();
  
  // Ensure 2 digits
  if (checksumStr.length() == 1) {
    checksumStr = "0" + checksumStr;
  }
  
  return checksumStr;
}

// Send NMEA sentence via Serial1
void sendNMEASentence(String sentence) {
  // Send to UART
  Serial1.print(sentence);
  Serial1.print("\r\n");
  
  // Echo to debug serial
  Serial.print("[SENT] ");
  Serial.println(sentence);
  
  // Show statistics every 10 sentences
  if (sentenceCounter % 10 == 0 && sentenceCounter > 0) {
    Serial.print("\n[STATS] Sent ");
    Serial.print(sentenceCounter);
    Serial.print(" sentences, Position: ");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.print(longitude, 6);
    Serial.print(", Alt: ");
    Serial.print(altitude, 1);
    Serial.println("m\n");
  }
}
/*
 * NMEA Generator for RP2040-Tiny
 * Outputs NMEA data on GPIO0 (TX)
 * 
 * Connection to RP2350-Zero converter:
 * RP2040-Tiny GPIO0 (Pin 0 right side) -> RP2350-Zero GPIO1 (Pin 1 right side)
 * RP2040-Tiny GND -> RP2350-Zero GND
 */

// Configuration
#define NMEA_INTERVAL 1000  // Send NMEA every 1 second
#define SHOW_DEBUG true     // Show what we're sending via USB

// Test NMEA sentences
const char* nmeaSentences[] = {
  "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
  "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
  "$GPGSV,3,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75",
  "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
};

int sentenceIndex = 0;
unsigned long lastSendTime = 0;
int sentCount = 0;

void setup() {
  // Initialize UART output on GPIO0/GPIO1
  Serial1.setTX(0);  // GPIO0 as TX
  Serial1.setRX(1);  // GPIO1 as RX (not used)
  Serial1.begin(115200);
  
  if (SHOW_DEBUG) {
    // USB Serial for debug (optional)
    Serial.begin(115200);
    
    // Wait a bit for USB to connect (optional)
    delay(2000);
    
    Serial.println("\n=====================================");
    Serial.println("    NMEA Generator for RP2040-Tiny  ");
    Serial.println("=====================================");
    Serial.println("\nConfiguration:");
    Serial.println("- TX Output: GPIO0 (Pin 0 right side)");
    Serial.println("- Baud Rate: 115200");
    Serial.println("- Interval: 1 second");
    Serial.println("\nConnect GPIO0 to converter's GPIO1");
    Serial.println("\nStarting NMEA transmission...\n");
  }
  
  // Small delay before starting
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  
  // Send NMEA sentence every interval
  if (now - lastSendTime >= NMEA_INTERVAL) {
    lastSendTime = now;
    
    // Get next sentence
    const char* sentence = nmeaSentences[sentenceIndex];
    sentenceIndex = (sentenceIndex + 1) % 4;
    
    // Send via UART on GPIO0
    Serial1.print(sentence);
    Serial1.print("\r\n");
    
    sentCount++;
    
    // Debug output (if enabled)
    if (SHOW_DEBUG && Serial) {
      Serial.print("[");
      Serial.print(sentCount);
      Serial.print("] TX -> ");
      Serial.println(sentence);
      
      // Status every 10 sentences
      if (sentCount % 10 == 0) {
        Serial.print("\n[STATUS] Sent ");
        Serial.print(sentCount);
        Serial.println(" NMEA sentences\n");
      }
    }
  }
}
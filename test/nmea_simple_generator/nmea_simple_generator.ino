/*
 * Simple NMEA Generator - No USB Serial Required
 * Just outputs NMEA on TX pin without any debug
 * 
 * For Waveshare boards:
 * Pin 2 (GPIO0) = TX output
 * Pin 1 (GND) = Ground
 */

void setup() {
  // Initialize only Serial1 for NMEA output
  Serial1.begin(115200);
  
  // Small delay for stability
  delay(1000);
}

void loop() {
  // Send simple NMEA sentences every second
  
  // GPGGA
  Serial1.println("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  delay(250);
  
  // GPRMC
  Serial1.println("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
  delay(250);
  
  // GPGSV
  Serial1.println("$GPGSV,3,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75");
  delay(250);
  
  // GPVTG
  Serial1.println("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48");
  delay(250);
}
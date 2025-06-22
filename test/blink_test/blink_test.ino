/*
 * SUPER SIMPLE BLINK TEST
 * Just blinks and prints to check if board is working
 */

void setup() {
  Serial.begin(115200);
  
  // Wait for USB
  while (!Serial && millis() < 5000) {
    delay(100);
  }
  
  Serial.println("\n\n===== BLINK TEST STARTED =====");
  Serial.println("This is BLINK TEST firmware");
  Serial.println("NOT the NMEA converter!");
  Serial.println("=============================\n");
  
  pinMode(0, OUTPUT);  // GPIO0
  pinMode(1, INPUT);   // GPIO1
}

void loop() {
  static int count = 0;
  
  // Print counter
  Serial.print("Loop #");
  Serial.print(count++);
  Serial.print(" - ");
  
  // Blink GPIO0
  digitalWrite(0, HIGH);
  Serial.print("GPIO0=HIGH, GPIO1=");
  Serial.print(digitalRead(1));
  
  delay(500);
  
  digitalWrite(0, LOW);
  Serial.print(" | GPIO0=LOW, GPIO1=");
  Serial.println(digitalRead(1));
  
  delay(500);
}
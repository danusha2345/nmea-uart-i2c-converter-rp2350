/*
 * Simplest GPIO Test for RP2350-Zero
 * Blinks GPIO0 and reads GPIO1
 */

// Explicit GPIO definitions
#define GPIO0 0
#define GPIO1 1

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  
  Serial.println("\n=== Simple GPIO0/GPIO1 Test ===");
  
  // Configure pins
  pinMode(GPIO0, OUTPUT);
  pinMode(GPIO1, INPUT_PULLDOWN);  // Pull down so we see clear HIGH/LOW
  
  Serial.println("GPIO0 = OUTPUT (will blink)");
  Serial.println("GPIO1 = INPUT with pulldown");
  Serial.println("\nConnect GPIO0 to GPIO1 to see GPIO1 follow GPIO0\n");
  
  delay(1000);
}

void loop() {
  // Set GPIO0 HIGH
  digitalWrite(GPIO0, HIGH);
  Serial.print("GPIO0 = HIGH, GPIO1 = ");
  delay(50); // Small delay for signal to settle
  Serial.println(digitalRead(GPIO1) ? "HIGH ✓" : "LOW");
  
  delay(1000);
  
  // Set GPIO0 LOW  
  digitalWrite(GPIO0, LOW);
  Serial.print("GPIO0 = LOW,  GPIO1 = ");
  delay(50);
  Serial.println(digitalRead(GPIO1) ? "HIGH" : "LOW ✓");
  
  delay(1000);
  Serial.println("----------");
}
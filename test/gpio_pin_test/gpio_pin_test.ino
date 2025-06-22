/*
 * GPIO Pin Test for RP2350-Zero
 * Tests GPIO0 and GPIO1 as digital pins
 * 
 * First test WITHOUT wire - pins should toggle independently
 * Then connect GPIO0 to GPIO1 - GPIO1 should follow GPIO0
 */

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n=== GPIO Pin Test ===");
  Serial.println("Testing GPIO0 and GPIO1");
  Serial.println("First run WITHOUT wire connection");
  Serial.println("Then connect GPIO0 to GPIO1\n");
  
  // Configure pins
  pinMode(0, OUTPUT);  // GPIO0 as output
  pinMode(1, INPUT);   // GPIO1 as input
  
  delay(1000);
}

void loop() {
  // Test 1: Set GPIO0 HIGH
  Serial.print("Setting GPIO0 = HIGH... ");
  digitalWrite(0, HIGH);
  delay(100);
  
  int gpio1State = digitalRead(1);
  Serial.print("GPIO1 reads: ");
  Serial.print(gpio1State ? "HIGH" : "LOW");
  
  if (gpio1State == HIGH) {
    Serial.println(" <- Wire connected!");
  } else {
    Serial.println(" <- No connection");
  }
  
  delay(1000);
  
  // Test 2: Set GPIO0 LOW
  Serial.print("Setting GPIO0 = LOW... ");
  digitalWrite(0, LOW);
  delay(100);
  
  gpio1State = digitalRead(1);
  Serial.print("GPIO1 reads: ");
  Serial.print(gpio1State ? "HIGH" : "LOW");
  
  if (gpio1State == LOW) {
    Serial.println(" <- Wire connected!");
  } else {
    Serial.println(" <- No connection");
  }
  
  Serial.println("-------------------");
  delay(2000);
}

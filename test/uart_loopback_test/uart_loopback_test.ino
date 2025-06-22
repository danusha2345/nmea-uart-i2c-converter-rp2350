/*
 * Simple UART Loopback Test for RP2350-Zero
 * Tests if UART is working correctly
 * 
 * Connect GPIO0 to GPIO1 with a wire
 */

void setup() {
  // USB Serial for debug
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=== UART Loopback Test ===");
  Serial.println("Connect GPIO0 to GPIO1");
  
  // Configure UART1 on specific pins
  Serial1.setTX(0);  // GPIO0 as TX
  Serial1.setRX(1);  // GPIO1 as RX
  Serial1.begin(115200);
  
  Serial.println("UART configured:");
  Serial.println("- TX on GPIO0");
  Serial.println("- RX on GPIO1");
  Serial.println("\nStarting test...\n");
  
  delay(1000);
}

void loop() {
  static int counter = 0;
  
  // Send test message
  String testMsg = "Test #" + String(counter++);
  Serial.print("Sending: ");
  Serial.println(testMsg);
  
  // Send via UART
  Serial1.println(testMsg);
  
  // Wait a bit
  delay(100);
  
  // Check if we received anything
  String received = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    received += c;
  }
  
  if (received.length() > 0) {
    Serial.print("Received: ");
    Serial.print(received);
    Serial.println(" ✓ UART WORKING!");
  } else {
    Serial.println("No data received ✗");
  }
  
  // Also test direct character echo
  Serial.println("\nDirect echo test - sending 'ABC'");
  Serial1.print('A');
  delay(10);
  if (Serial1.available()) {
    Serial.print("Got: ");
    Serial.println((char)Serial1.read());
  }
  
  Serial1.print('B');
  delay(10);
  if (Serial1.available()) {
    Serial.print("Got: ");
    Serial.println((char)Serial1.read());
  }
  
  Serial1.print('C');
  delay(10);
  if (Serial1.available()) {
    Serial.print("Got: ");
    Serial.println((char)Serial1.read());
  }
  
  Serial.println("\n-------------------");
  delay(2000);
}
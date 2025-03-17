HardwareSerial mySerial(2);

#define CRSF_SYNC_BYTE 0xEE  // RC packets (TX → RX)

void setup() {
  Serial.begin(115200);
  mySerial.begin(400000, SERIAL_8N1, 16, 17, true);  // UART2 (RX=16, TX=17)
}

void loop() {
  static uint8_t buffer[64];  // Buffer to store incoming data
  static uint8_t bufferIndex = 0;
  
  while (mySerial.available()) {
    uint8_t byteIn = mySerial.read();

    // Look for a valid start of a CRSF frame
    if (bufferIndex == 0) {
      if (byteIn == CRSF_SYNC_BYTE) {
        buffer[bufferIndex++] = byteIn;  // Store valid address byte
      }
    } 
    else if (bufferIndex == 1) {  
      // Read the length byte (payload size + 2 for address & CRC)
      if (byteIn < sizeof(buffer)) {  // Ensure length is valid
        buffer[bufferIndex++] = byteIn;
      } else {
        bufferIndex = 0;  // Reset if invalid length
      }
    } 
    else {  
      // Read the remaining bytes
      buffer[bufferIndex++] = byteIn;

      // Check if we have received the full frame
      if (bufferIndex >= buffer[1] + 2) {  
        // Full packet received, print first 4 bytes
        Serial.print(buffer[0], HEX); Serial.print(" ");
        Serial.print(buffer[1]); Serial.print(" ");
        Serial.print(buffer[2], HEX); Serial.print(" ");
        Serial.print(buffer[3], HEX); Serial.print(" ");
        Serial.print(buffer[4], HEX); Serial.print(" ");
        Serial.print(buffer[5], HEX);Serial.print(" ");
        Serial.print(buffer[6], HEX); Serial.print(" ");
        Serial.print(buffer[7], HEX);Serial.print(" ");
        Serial.print(buffer[8], HEX); Serial.print(" ");
        Serial.println(buffer[9], HEX);Serial.print(" ");

        // Reset buffer index for next frame
        bufferIndex = 0;
      }
    }
  }
}

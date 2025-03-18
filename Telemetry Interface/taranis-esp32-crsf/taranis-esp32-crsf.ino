HardwareSerial mySerial(2);

#define CRSF_SYNC_BYTE 0xEE  // RC packets (TX → RX)

uint16_t channels[16];

void setup() {
  Serial.begin(115200);
  mySerial.begin(400000, SERIAL_8N1, 16, 17, true);  // UART2 (RX=16, TX=17)
}

void loop() {
  uint8_t buffer[64];  // Buffer to store incoming data
  uint8_t bufferIndex = 0;
  
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
        /*Serial.print(buffer[0], HEX); Serial.print(" ");    // Serial Sync Byte
        Serial.print(buffer[1], HEX); Serial.print(" ");    // Frame Length

        Serial.print(buffer[2], HEX); Serial.print(" ");    // Type

        Serial.print(((uint16_t)buffer[3] | ((uint16_t)(buffer[4] & 0x07) << 8)), DEC); Serial.print(" ");
        Serial.print(((uint16_t)(buffer[4] >> 3) | ((uint16_t)(buffer[5] & 0x3F) << 5)), DEC);Serial.print(" ");
        Serial.print(((uint16_t)(buffer[5] >> 6) | ((uint16_t)buffer[6] << 2) | ((uint16_t)(buffer[7] & 0x01) << 10)), DEC);Serial.print(" ");
        Serial.println(((uint16_t)(buffer[7] >> 1) | ((uint16_t)(buffer[8] & 0x0F) << 7)), DEC);Serial.print(" ");*/
        extractCRSFChannels(buffer, channels);
        for(int m = 0; m < 15; m++){
          Serial.print(channels[m]);
          Serial.print(" ");
        }
        Serial.println(channels[15]);

        // Reset buffer index for next frame
        bufferIndex = 0;
      }
    }
  }
}

void extractCRSFChannels(uint8_t *buffer, uint16_t *channels) {
    for (int i = 0; i < 16; i++) {
        int byteIndex = 3 + (i * 11) / 8;  // Start byte of this channel
        int bitShift = (i * 11) % 8;       // Bit offset within these bytes

        uint32_t value = ((uint32_t)buffer[byteIndex]) |
                         ((uint32_t)buffer[byteIndex + 1] << 8) |
                         ((uint32_t)buffer[byteIndex + 2] << 16);  // Ensure 3-byte storage

        channels[i] = (value >> bitShift) & 0x07FF;  // Extract 11 bits
    }
}

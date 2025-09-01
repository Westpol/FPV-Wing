#include <WiFi.h>

const char *ssid = "ESP32_Telemetry";
const char *password = "12345678";

HardwareSerial mySerial(2);

WiFiUDP udp;

#define CRSF_SYNC_BYTE 0xC8 // RC packets (TX → RX)
#define CRSF_PACKET_LENGTH 0x0A
#define CRSF_TYPE_BATTERY 0x08
#define CRSF_TYPE_FLIGHT_MODE 0x21
#define CRSF_TYPE_ATTITUDE 0x1E
#define CRSF_TYPE_GPS 0x02
#define CRSF_TYPE_BARO 0x09
#define CRSF_TYPE_VARIO 0x07

uint16_t channels[16];

uint8_t buffer[64];  // Buffer to store incoming data
uint8_t usedBufferLength = 0;
bool validFrame = false;
uint8_t bufferIndex = 0;

const uint16_t udpPort = 8888;
IPAddress broadcastIP(192,168,4,255);

void setup() {
  Serial.begin(115200);
  mySerial.begin(416666, SERIAL_8N1, 16, 17, true);  // UART2 (RX=16, TX=17)

  WiFi.softAP(ssid, password);

  Serial.println("Access Point started!");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP()); // default is usually 192.168.4.1

  udp.begin(udpPort);
}

void loop() {
  if(validFrame && bufferIndex == 0){
    sendUDP(buffer, usedBufferLength);
    validFrame = false;
  }
  while (mySerial.available()) {
    uint8_t byteIn = mySerial.read();
    if(bufferIndex > 0){
      printHex(byteIn);
      bufferIndex--;
      buffer[usedBufferLength++] = byteIn;
      if(usedBufferLength > 60){
        return;
      }
    }
    if(byteIn == CRSF_SYNC_BYTE){
      while (!mySerial.available()) {}
      uint8_t lengthByte = mySerial.read();
      if(lengthByte > 30) return;
      while (!mySerial.available()) {}
      uint8_t typeByte = mySerial.read();
      if(typeByte == CRSF_TYPE_FLIGHT_MODE || typeByte == CRSF_TYPE_BATTERY || typeByte == CRSF_TYPE_ATTITUDE || typeByte == CRSF_TYPE_BARO || typeByte == CRSF_TYPE_GPS || typeByte == CRSF_TYPE_VARIO){
      bufferIndex = lengthByte - 1;
      Serial.println("");
      printHex(byteIn);
      printHex(lengthByte);
      printHex(typeByte);
      buffer[0] = byteIn;
      buffer[1] = lengthByte;
      buffer[2] = typeByte;
      usedBufferLength = 3;
      validFrame = true;
      }
      else{
        return;
      }
    }
  }
}

void sendUDP(uint8_t *data, size_t len) {
  udp.beginPacket(broadcastIP, udpPort);
  udp.write(data, len);
  udp.endPacket();
}

void printHex(uint8_t hex_value){
  if(hex_value == 0){
    Serial.print("00");
  }
  else if(hex_value < 16){
    Serial.print(hex_value < 16 ? "0" : "");
    Serial.print(hex_value, HEX);
  }
  else {
    Serial.print(hex_value, HEX);
  }
  Serial.print(" ");
}
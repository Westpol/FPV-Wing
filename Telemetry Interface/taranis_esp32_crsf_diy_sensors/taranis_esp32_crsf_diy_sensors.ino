#include <WiFi.h>

const char *ssid = "ESP32_Telemetry";
const char *password = "12345678";

HardwareSerial mySerial(2);

WiFiUDP udp;

#define CRSF_SYNC_BYTE 0xC8 // RC packets (TX → RX)
#define CRSF_PACKET_LENGTH 0x0A

uint16_t channels[16];

uint8_t buffer[64];  // Buffer to store incoming data
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
  while (mySerial.available()) {
    uint8_t byteIn = mySerial.read();
    if(bufferIndex > 0){
      printHex(byteIn);
      bufferIndex--;
    }
    if(byteIn == CRSF_SYNC_BYTE){
      while (!mySerial.available()) {}
      uint8_t byte2In = mySerial.read();
      if(byte2In == CRSF_PACKET_LENGTH){
      bufferIndex = CRSF_PACKET_LENGTH;
      Serial.println("");
      printHex(byteIn);
      printHex(byte2In);
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
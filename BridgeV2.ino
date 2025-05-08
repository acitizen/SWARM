#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>  // Note the capital UDP

// WiFi credentials
const char* ssid = "SPACETIME.IO";
const char* password = "Zatawaxaqe";

// UDP settings
WiFiUDP udp;  // Note the capital UDP
const uint16_t UDP_PORT = 8888;
char packetBuffer[512];

// Bridge IP address
IPAddress bridge_ip(192, 168, 20, 23);
IPAddress gateway(192, 168, 20, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(2000); 
  Serial.println("UWB Positioning Bridge starting...");
  
  // Set static IP before connecting to WiFi
  if (!WiFi.config(bridge_ip, gateway, subnet)) {
    Serial.println("Static IP configuration failed");
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Bridge IP address: ");
  Serial.println(WiFi.localIP());
  
  // Begin listening for UDP packets
  udp.begin(UDP_PORT);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);
  
  // Add a simple indicator to show bridge is working
  for (int i = 0; i < 5; i++) {
    Serial.println("Bridge ready and waiting for packets...");
    delay(500);
  }
}

void loop() {
  // Check if there are any UDP packets available
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    // Read the packet into the buffer
    int len = udp.read(packetBuffer, sizeof(packetBuffer));
    
    if (len > 0) {
      packetBuffer[len] = 0; // Null-terminate the string
      
      // Print sender info and data
      Serial.print("Received packet from ");
      Serial.print(udp.remoteIP());
      Serial.print(":");
      Serial.print(udp.remotePort());
      Serial.print(" - Length: ");
      Serial.println(len);
      
      // Forward the packet content to the serial port without extra info
      // for visualization software to parse
      Serial.println(packetBuffer);
    }
  }
  
  // Add a periodic heartbeat to show the bridge is running
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    Serial.print("Bridge waiting for packets... IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // Brief delay
  delay(10);
}
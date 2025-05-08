#include "dw3000.h"
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>  // Note the capital UDP

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

// Tag ID
#define TAG_ID 0

// WiFi credentials
const char* ssid = "SPACETIME.IO";
const char* password = "Zatawaxaqe";

// UDP settings for sending to bridge
WiFiUDP udp;  // Note the capital UDP here
IPAddress bridgeIP(192, 168, 20, 23);  // Bridge IP
const uint16_t UDP_PORT = 8888;

#define RNG_DELAY_MS 50  // Reduced for more frequent ranging
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 14
#define RESP_MSG_RESP_TX_TS_IDX 18
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 800  // Doubled from 400

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

// Custom message formats
#define POLL_MSG 0x01     // Poll message from tag
#define RESP_MSG 0x02     // Response from anchor with position info

static uint8_t tx_poll_msg[12] = {0};  // Poll message to send
static uint8_t rx_resp_msg[32] = {0};  // Buffer for response messages
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[32];
static uint32_t status_reg = 0;
static double tag_distance;
// Using a different name to avoid conflict with the library's dist_str
static char distance_str[20];

// Anchor data
typedef struct {
  int32_t x, y, z;    // Position coordinates
  float distance;     // Measured distance
  bool updated;       // Whether we have an updated distance
} anchor_data_t;

anchor_data_t anchors[4];  // Data for 4 anchors

// Tag position (calculated)
float tag_x = 0, tag_y = 0, tag_z = 0;

extern dwt_txconfig_t txconfig_options;

// Function prototypes
void calculatePosition();
void sendPositionData();
// No need to declare resp_msg_get_ts as we'll use the library's version

void setup()
{
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  Serial.print("Initializing Tag ");
  Serial.println(TAG_ID);
  
  // Setup static IP before connecting to WiFi
  IPAddress tag_ip(192, 168, 20, 22);    // Static IP for tag
  IPAddress gateway(192, 168, 20, 1);    // Router IP (adjust if different)
  IPAddress subnet(255, 255, 255, 0);    // Standard subnet mask
  
  if (!WiFi.config(tag_ip, gateway, subnet)) {
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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Initialize DW3000
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("IDLE FAILED");
    delay(100);
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    Serial.println("INIT FAILED");
    while (1);
  }

  // Enable LEDs for debugging
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC
  if (dwt_configure(&config))
  {
    Serial.println("CONFIG FAILED");
    while (1);
  }

  // Configure the TX spectrum parameters
  dwt_configuretxrf(&txconfig_options);

  // Apply default antenna delay values
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  // Set expected response's delay and timeout
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // Enable TX/RX LEDs for debug
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // Initialize the poll message
  tx_poll_msg[0] = POLL_MSG;      // Message type
  tx_poll_msg[1] = TAG_ID;        // Tag ID
  
  // Print the poll message bytes for verification
  Serial.print("Poll message headers: 0x");
  Serial.print(tx_poll_msg[0], HEX);
  Serial.print(" 0x");
  Serial.print(tx_poll_msg[1], HEX);
  Serial.println();
  // Initialize anchor data
  for (int i = 0; i < 4; i++) {
    anchors[i].updated = false;
    anchors[i].distance = 0;
  }
  
  Serial.print("Tag ");
  Serial.print(TAG_ID);
  Serial.println(" initialized and ready");
}

 void loop()
{
  // Reset all anchor updates for new cycle
  for (int i = 0; i < 4; i++) {
    anchors[i].updated = false;
  }
  
  // Range with each anchor
  for (int anchor_id = 0; anchor_id < 4; anchor_id++) {
    // Debug output before ranging
    Serial.print("Attempting to range with Anchor ");
    Serial.println(anchor_id);
    
    // Set the target anchor ID
    tx_poll_msg[2] = anchor_id;
  
    // Write frame data to DW IC and prepare transmission
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(3, tx_poll_msg, 0); // 3 bytes - type, tag id, anchor id
    dwt_writetxfctrl(3, 0, 1);          // Zero offset in TX buffer, ranging

    // Start transmission, indicating that a response is expected
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    Serial.println("Poll message sent");

    // Poll for reception of a frame or error/timeout
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    // Increment frame sequence number after transmission of the poll message
    frame_seq_nb++;
    
    // Check if we received a valid frame
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
      // Clear good RX frame event in the DW IC status register
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      // Read received frame data
      uint32_t frameLen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
      if (frameLen <= sizeof(rx_buffer))
      {
        dwt_readrxdata(rx_buffer, frameLen, 0);

        // Debug - print received data (first 12 bytes)
        Serial.print("Received data: ");
        for (int i = 0; i < frameLen && i < 12; i++) {
          Serial.print(rx_buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Check if the frame is a response from the correct anchor
        if (rx_buffer[0] == RESP_MSG && rx_buffer[1] == anchor_id)
        {
          Serial.print("Received valid response from anchor ");
          Serial.println(anchor_id);
          
          uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
          int32_t rtd_init, rtd_resp;
          float clockOffsetRatio;

          // Extract anchor position
          int32_t ax, ay, az;
          memcpy(&ax, &rx_buffer[2], 4);
          memcpy(&ay, &rx_buffer[6], 4);
          memcpy(&az, &rx_buffer[10], 4);
          
          // Store anchor position
          anchors[anchor_id].x = ax;
          anchors[anchor_id].y = ay;
          anchors[anchor_id].z = az;

          // Retrieve poll transmission and response reception timestamps
          poll_tx_ts = dwt_readtxtimestamplo32();
          resp_rx_ts = dwt_readrxtimestamplo32();

          // Read carrier integrator value and calculate clock offset ratio
          clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

          // Get timestamps embedded in response message
          // Using the library's resp_msg_get_ts function
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          // Compute time of flight and distance
          rtd_init = resp_rx_ts - poll_tx_ts;
          rtd_resp = resp_tx_ts - poll_rx_ts;

          float tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
          float dist = tof * SPEED_OF_LIGHT;

          // Store distance
          anchors[anchor_id].distance = dist;
          anchors[anchor_id].updated = true;
          
          // Output to serial for debugging
          snprintf(distance_str, sizeof(distance_str), "A%d DIST: %3.2f m", anchor_id, dist);
          Serial.println(distance_str);
        }
        else {
          Serial.print("Received invalid response. Type: 0x");
          Serial.print(rx_buffer[0], HEX);
          Serial.print(", From: ");
          Serial.println(rx_buffer[1]);
        }
      }
    }
    else
    {
      // Get detailed error information
      uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
      
      // Clear RX error/timeout events
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      
      Serial.print("Error ranging with Anchor ");
      Serial.print(anchor_id);
      Serial.print(", Status: 0x");
      Serial.println(status, HEX);
      
      // Check specific errors
      if (status & SYS_STATUS_RXFTO_BIT_MASK) {
        Serial.println("  Receive frame timeout");
      }
      if (status & SYS_STATUS_RXPHE_BIT_MASK) {
        Serial.println("  Receive PHY header error");
      }
      if (status & SYS_STATUS_RXFCE_BIT_MASK) {
        Serial.println("  Receive frame CRC error");
      }
      if (status & SYS_STATUS_RXFSL_BIT_MASK) {
        Serial.println("  Receive frame sync loss");
      }
      if (status & SYS_STATUS_RXSTO_BIT_MASK) {
        Serial.println("  Receive SFD timeout");
      }
    }
    
    // Add a short delay between ranging exchanges
    delay(10);
  }
  
  // Calculate position if we have enough anchor measurements
  int valid_anchors = 0;
  for (int i = 0; i < 4; i++) {
    if (anchors[i].updated) {
      valid_anchors++;
    }
  }
  
  Serial.print("Valid anchors: ");
  Serial.println(valid_anchors);
  
  if (valid_anchors >= 3) {
    calculatePosition();
    sendPositionData();
  } else {
    Serial.println("Not enough valid anchor measurements for positioning");
  }
  
  // Delay before next ranging cycle
  delay(RNG_DELAY_MS);
}

// Calculate position using trilateration (multilateration)
void calculatePosition() {
  // Find three valid anchor measurements
  int valid_indices[3] = {-1, -1, -1};
  int count = 0;
  
  for (int i = 0; i < 4 && count < 3; i++) {
    if (anchors[i].updated) {
      valid_indices[count++] = i;
    }
  }
  
  if (count < 3) {
    Serial.println("Not enough valid anchor measurements");
    return;
  }
  
  // Get the three anchor positions and distances
  float x1 = anchors[valid_indices[0]].x;
  float y1 = anchors[valid_indices[0]].y;
  float z1 = anchors[valid_indices[0]].z;
  float r1 = anchors[valid_indices[0]].distance;
  
  float x2 = anchors[valid_indices[1]].x;
  float y2 = anchors[valid_indices[1]].y;
  float z2 = anchors[valid_indices[1]].z;
  float r2 = anchors[valid_indices[1]].distance;
  
  float x3 = anchors[valid_indices[2]].x;
  float y3 = anchors[valid_indices[2]].y;
  float z3 = anchors[valid_indices[2]].z;
  float r3 = anchors[valid_indices[2]].distance;
  
  // Trilateration calculation
  // Based on the solution of the intersection of three spheres
  float A = 2*(x2 - x1);
  float B = 2*(y2 - y1);
  float C = 2*(z2 - z1);
  float D = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2 - z1*z1 + z2*z2;
  
  float E = 2*(x3 - x2);
  float F = 2*(y3 - y2);
  float G = 2*(z3 - z2);
  float H = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3 - z2*z2 + z3*z3;
  
  // Check for division by zero
  float denominator = (A*F*G - B*E*G - A*G*F + C*E*F);
  if (abs(denominator) < 0.001) {
    Serial.println("Trilateration calculation error - singular matrix");
    return;
  }
  
  // Solve for z
  float z = (A*F*H - B*E*H - A*G*D + C*E*D) / denominator;
  
  // Solve for y
  float y = 0;
  if (abs(F) > 0.001) {
    y = (H - G*z) / F;
  } else if (abs(B) > 0.001) {
    y = (D - C*z) / B;
  }
  
  // Solve for x
  float x = 0;
  if (abs(A) > 0.001) {
    x = (D - B*y - C*z) / A;
  } else if (abs(E) > 0.001) {
    x = (H - F*y - G*z) / E;
  }
  
  // Update tag position
  tag_x = x;
  tag_y = y;
  tag_z = z;
  
  Serial.print("Tag position: (");
  Serial.print(tag_x);
  Serial.print(", ");
  Serial.print(tag_y);
  Serial.print(", ");
  Serial.print(tag_z);
  Serial.println(")");
}

// Send position data to bridge via UDP
void sendPositionData() {
  // Create JSON-like string with position data
  String json = "{\"tag_id\":" + String(TAG_ID) + 
                ",\"x\":" + String(tag_x) + 
                ",\"y\":" + String(tag_y) + 
                ",\"z\":" + String(tag_z) + 
                ",\"anchors\":[";
  
  for (int i = 0; i < 4; i++) {
    json += "{\"id\":" + String(i) + 
            ",\"x\":" + String(anchors[i].x) + 
            ",\"y\":" + String(anchors[i].y) + 
            ",\"z\":" + String(anchors[i].z) + 
            ",\"distance\":" + String(anchors[i].updated ? anchors[i].distance : 0) + 
            ",\"valid\":" + String(anchors[i].updated ? "true" : "false") + "}";
    
    if (i < 3) {
      json += ",";
    }
  }
  
  json += "]}";
  
  // Send via UDP with debug info
  Serial.print("Sending to Bridge IP: ");
  Serial.print(bridgeIP);
  Serial.print(" Port: ");
  Serial.println(UDP_PORT);
  
  udp.beginPacket(bridgeIP, UDP_PORT);
  udp.print(json);
  bool success = udp.endPacket();
  
  Serial.print("UDP send result: ");
  Serial.println(success ? "Success" : "Failed");
  Serial.println(json);
}
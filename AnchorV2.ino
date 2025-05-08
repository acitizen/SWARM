#include "dw3000.h"
#include <SPI.h>

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

// Anchor ID (change for each anchor: 0-3)
#define ANCHOR_ID 0

// Anchor positions in mm (change these to your actual measured positions)
static int32_t ANCHOR_POSITIONS[4][3] = {
  {0, 0, 0},         // Anchor 0 position (x,y,z)
  {5000, 0, 0},      // Anchor 1 position
  {5000, 5000, 0},   // Anchor 2 position
  {0, 5000, 0}       // Anchor 3 position
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_TS_LEN 4
#define POLL_RX_TO_RESP_TX_DLY_UUS 450

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

// Custom message formats for anchor-tag ranging
#define POLL_MSG 0x01     // Poll message from tag
#define RESP_MSG 0x02     // Response from anchor with position info

static uint8_t rx_poll_msg[20] = {0};  // Will hold poll message from tag
static uint8_t tx_resp_msg[32] = {0};  // Response message with anchor position
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[32];
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern dwt_txconfig_t txconfig_options;

void setup()
{
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  Serial.print("Initializing Anchor ");
  Serial.println(ANCHOR_ID);
  
  // Initialize SPI
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

  // Enable TX/RX LEDs for debug
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // Initialize the anchor response message
  tx_resp_msg[0] = RESP_MSG;               // Message type
  tx_resp_msg[1] = ANCHOR_ID;              // Anchor ID
  
  // Add anchor position to the response message
  memcpy(&tx_resp_msg[2], &ANCHOR_POSITIONS[ANCHOR_ID][0], 12);  // 12 bytes for x,y,z (4 bytes each)
  
  Serial.print("Anchor ");
  Serial.print(ANCHOR_ID);
  Serial.println(" initialized and ready");
  Serial.print("Position: (");
  Serial.print(ANCHOR_POSITIONS[ANCHOR_ID][0]);
  Serial.print(", ");
  Serial.print(ANCHOR_POSITIONS[ANCHOR_ID][1]);
  Serial.print(", ");
  Serial.print(ANCHOR_POSITIONS[ANCHOR_ID][2]);
  Serial.println(")");
}

void loop()
{
  // Activate reception immediately
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Poll for reception of a frame or error
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    // Clear good RX frame event in the DW IC status register
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    // Read received frame data
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      // Check if the received message is a poll message from a tag
      if (rx_buffer[0] == POLL_MSG)
      {
        uint8_t tag_id = rx_buffer[1];
        uint32_t resp_tx_time;
        int ret;

        // Retrieve poll reception timestamp - use library's function
        poll_rx_ts = get_rx_timestamp_u64();

        // Compute response message transmission time
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        // Response TX timestamp is the transmission time we programmed plus the antenna delay
        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        // Write timestamps in the response message - use library's function
        resp_msg_set_ts(&tx_resp_msg[14], poll_rx_ts);  // Index 14 for poll_rx timestamp
        resp_msg_set_ts(&tx_resp_msg[18], resp_tx_ts);  // Index 18 for resp_tx timestamp

        // Update sequence number
        tx_resp_msg[22] = frame_seq_nb;
        
        // Write and send the response message
        dwt_writetxdata(24, tx_resp_msg, 0);  // 24 bytes: type, id, pos(12), timestamps(8), seq
        dwt_writetxfctrl(24, 0, 1);          // 24 bytes, no offset, ranging bit set
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        // If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one
        if (ret == DWT_SUCCESS)
        {
          // Poll DW IC until TX frame sent event set
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));

          // Clear TXFRS event
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

          // Increment frame sequence number (modulo 256)
          frame_seq_nb++;
          
          Serial.print("Responded to Tag ");
          Serial.println(tag_id);
        }
      }
    }
  }
  else
  {
    // Clear RX error events
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    
    // For RX reset, use dwt_forcetrxoff() to force the transceiver off
    dwt_forcetrxoff();
  }
  // DEBUG In the anchor's loop function
if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
{
  uint32_t frame_len;
  
  // Clear good RX frame event
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  
  // Read received frame data
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len <= sizeof(rx_buffer))
  {
    dwt_readrxdata(rx_buffer, frame_len, 0);
    
    // Debug output - print all received frames
    Serial.print("Received frame type: 0x");
    Serial.print(rx_buffer[0], HEX);
    Serial.print(", From Tag: ");
    Serial.println(rx_buffer[1]);
    
    // Check if it's a poll message
    if (rx_buffer[0] == POLL_MSG)
    {
      // Process poll message as before...
      Serial.println("Processing POLL message");
      
      // After sending response
      Serial.println("Response sent to tag");
    }
  }
}
else
{
  // Print error status for debugging
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
  Serial.print("Error status: 0x");
  Serial.println(status, HEX);
  
  // Clear errors
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
}
}
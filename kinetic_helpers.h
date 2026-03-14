#include <AceCRC.h>

// Select the type of CRC algorithm we'll be using
using namespace ace_crc::crc16ccitt_byte;

// I've tested three device types so far
// Self powered switch only devices, 5 byte message, two byte device ID
// Switch and relay devices, 10 byte message, four byte device ID (that seem to start with 0xB1 from a small sample size)
// Relay only devices/smart plugs , 10 byte message, four byte device ID (that seem to start with 0x004B from a small sample size)
// N.B. The label on the device has the full device id, except for the smart plugs for some reason.

enum deviceType {
  SWITCH_ONLY,
  SWITCH_RELAY,
  RELAY,
  UNKNOWN,
};

// Poll is 0x0D, device returns 0x0D
// State reports come from device only with 0x0C
// Set is 0x0B, device returns 0x0C (a state report)

enum messageType {
  POLL,
  STATE,
  SET,
  MSG_UNKNOWN,
};

// Message definition struct
// Device ID is two byes for switch only, four bytes for switch relay and relays, not got other types to test
// The device type can be detected, as it seems to be indicated by a mixture of message length and the first byte of the device ID
// I'm unsure if we can rely on this for other device types, so an explicit definition field is included
// Message type indicates whether the message is a poll from the device to get device state, 
// state report (sent by the device when it changes and distinct from a poll), or set command from the hub to the device
// State is one byte, for relay devices 0x00 is off, 0x01 is on (on single channel relays)
// Single channel switch only devices send 0xc0 for release and 0x04 for press
// Multu channel devices are untested by me but allegedly use other bits in the byte for the button number

struct kineticMessage {
  uint32_t deviceID;
  deviceType devType;
  messageType msgType;
  uint8_t state;
};


void encodeForTransmission(uint8_t *outBuf, uint32_t deviceID, messageType msgType, bool state){
  // First 4 bytes are device ID
  // 5th Byte is 0x0D if polling, 0x0B if setting
  // 6th Byte is 0x00 if polling, 0x01 if setting
  // 7th Byte is 0x55 if polling, 0x07 if setting a receiver switch, 0x02 if setting a relay (IDK why) 
  // (Relay device IDs seem to start with 0x00 based on a sample size of 4 ^w^)
  // 8th byte is 0xAA for polling, 0x00 to set off, 0x01 to set on
  // Final two bytes are CRC16/AUG-CCITT of first 8 bytes
  uint8_t txBuf[10] = {(deviceID >> 24) & 0xFF,
                      (deviceID >> 16) & 0xFF,
                      (deviceID >> 8) & 0xFF,
                      (deviceID >> 0) & 0xFF,
                      (msgType == POLL) ? 0x0D : 0x0B,
                      (msgType == POLL) ? 0x00 : 0x01,
                      (msgType == POLL) ? 0x55 : ((deviceID >> 24) & 0xFF) ? 0x07 : 0x02,
                      (msgType == POLL) ? 0xAA : (state ? 0x01 : 0x00),
                      0x00,
                      0x00};

  // Calculate CRC and populate final bytes
  crc_t crc = crc_init();
  crc = crc_update(crc, txBuf, 8);
  crc = crc_finalize(crc);

  txBuf[8] = (crc >> 8) & 0xFF;
  txBuf[9] = crc & 0xFF;

  // Takes the 10 byte data packet and returns a 12 byte data packet with the bytes shifted 9 bits along and 0b000100011 inserted at the beginning
  // This accounts for the extra 0 bit in between preamble and actual sync word, and the third byte of the sync word, unsupported by cc1101
  for(int i=0; i<10; i++){
    outBuf[i+1] = (txBuf[i] >> 1) | (i==0 ? 0b10000000 : ((txBuf[i-1] & 0b00000001) << 7));
  }
  outBuf[0] = 0x11;
  outBuf[11] = (txBuf[9] << 7);
}

void unshiftTransmission(uint8_t *outBuf, uint8_t *inBuf){
  // Take the 12 byte input packet and deshift it, returning the original 10 byte packet
  for(int i=0; i<10; i++)
    outBuf[i] = (inBuf[i+1] << 1) | ((inBuf[i+2] >> 7) & 0b1);

}

struct kineticMessage* decodeTransmission(uint8_t *inBuf){
  static struct kineticMessage msg;

  // Check if message is a short 5 byte switch message
  uint16_t messageCRC = (inBuf[3] << 8) | inBuf[4];
  crc_t crc = crc_init();
  crc = crc_update(crc, inBuf, 3);
  crc = crc_finalize(crc);

  if(uint16_t(crc) == messageCRC){
    // Valid switch only message
    #ifdef DEBUG_SERIAL
    Serial.print("Switch detected crc");
    Serial.println(crc, HEX);
    #endif
    msg.deviceID = (inBuf[0] << 8) | inBuf[1];
    msg.devType = SWITCH_ONLY;
    msg.msgType = STATE; // Switch only devices only send state
    msg.state = inBuf[2];
    return &msg;
  }
  else {
    uint32_t messageCRC = (inBuf[8] << 8) | inBuf[9];
    crc_t crc = crc_init();
    crc = crc_update(crc, inBuf, 8);
    crc = crc_finalize(crc);
    if(uint16_t(crc) != messageCRC){
      // Invalid message
      Serial.println("Error: CRC Mismatch!");
      msg.deviceID = 0;
      msg.devType = UNKNOWN;
      msg.msgType = MSG_UNKNOWN;
      return &msg;
    } else {
      // Valid 10 byte message
        // First 4 bytes are device ID
      msg.deviceID = (inBuf[0] << 24) | (inBuf[1] << 16) | (inBuf[2] << 8) | inBuf[3];

      // Determine device type based on first byte of device ID
      if (((msg.deviceID >> 24) & 0xFF) == 0xB1) {
        msg.devType = SWITCH_RELAY;
      } else if (((msg.deviceID >> 24) & 0xFF) == 0x00) {
        msg.devType = RELAY;
      } else {
        msg.devType = UNKNOWN;
      }

      // Message type
      if (inBuf[4] == 0x0D) {
        msg.msgType = POLL;
      } else if (inBuf[4] == 0x0C || inBuf[4] == 0xA5) {
        msg.msgType = STATE;
      } else if (inBuf[4] == 0x0B) {
        msg.msgType = SET;
      } else {
        msg.msgType = MSG_UNKNOWN;
      }

      // State is in byte 7
      msg.state = inBuf[7];
      return &msg;
    }
  }


// Something has gone horribly wrong
  return nullptr;
}
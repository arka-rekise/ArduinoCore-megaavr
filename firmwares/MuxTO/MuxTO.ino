/*
   j2updi.cpp - No-reset USB stability fix
*/

// Includes
#include "sys.h"
#include "lock.h"
#include "updi_io.h"
#include "JICE_io.h"
#include "JTAG2.h"
#include "UPDI_hi_lvl.h"

volatile bool updi_mode = false;
unsigned long baudrate = 115200;
unsigned long updi_mode_start = 0;
unsigned long updi_mode_end = 0;
uint8_t serial_mode = SERIAL_8N1;
bool serialNeedReconfiguration = false;

char support_buffer[32];

struct lock q;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  JICE_io::init();
  UPDI_io::init();
  Serial1.begin(baudrate, serial_mode);
  lock_init(&q);
  JTAG2::sign_on();
}

void loop() {
  // Proper USB stability fix - prevents "device unrecognized" WITHOUT resetting
  // This clears the End of Reset flag which can cause USB enumeration issues
  // Does NOT trigger NVIC_SystemReset() - no unwanted resets!
  if (USB->DEVICE.INTFLAG.bit.EORST) {
    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
  }

  if (!updi_mode) {
    // Serial forwarding
    if (int c = Serial1.available()) {
      lock(&q);
      if (c > Serial.availableForWrite()) c = Serial.availableForWrite();
      unlock(&q);
      Serial1.readBytes(support_buffer, c);
      Serial.write(support_buffer, c);
    }

    if (int c = Serial.available()) {
      lock(&q);
      if (c > Serial1.availableForWrite()) c = Serial1.availableForWrite();
      unlock(&q);
      Serial.readBytes(support_buffer, c);
      Serial1.write(support_buffer, c);
    }

    // Handle baud rate changes
    uint32_t newBaud = Serial.baud();
    if (newBaud != baudrate) {
      baudrate = newBaud;
      if (Serial.dtr() == 1 && baudrate != 1200) {
        Serial1.end();
        Serial1.begin(baudrate, serial_mode);
        // Reset target chip via UPDI, not the programmer itself
        UPDI::stcs(UPDI::reg::ASI_Reset_Request, UPDI::RESET_ON);
        UPDI::stcs(UPDI::reg::ASI_Reset_Request, UPDI::RESET_OFF);
      }
    }

    // Enter UPDI mode at 1200 baud - target chip reset happens here
    // This does NOT reset the Nano Every itself, only signals UPDI mode
    if (baudrate == 1200 && Serial.dtr() == 0 && (millis() - updi_mode_end > 200)) {
      updi_mode = true;
      updi_mode_start = millis();
      updi_mode_end = 0;
    }
    return;
  }

  // UPDI mode timeout protection
  if ((updi_mode_end && (millis() - updi_mode_end) > 500) || ((millis() - updi_mode_start) > 60000)) {
    updi_mode = false;
    baudrate = 115200;
    return;
  }

  if (!JTAG2::receive()) return;

  switch (JTAG2::packet.body[0]) {
    case JTAG2::CMND_GET_SIGN_ON:
      JTAG2::sign_on();
      break;
    case JTAG2::CMND_GET_PARAMETER:
      JTAG2::get_parameter();
      break;
    case JTAG2::CMND_SET_PARAMETER:
      JTAG2::set_parameter();
      break;
    case JTAG2::CMND_RESET:
    case JTAG2::CMND_ENTER_PROGMODE:
      JTAG2::enter_progmode();
      break;
    case JTAG2::CMND_SIGN_OFF:
      JTAG2::PARAM_BAUD_RATE_VAL = JTAG2::baud_19200;
    case JTAG2::CMND_LEAVE_PROGMODE:
      JTAG2::leave_progmode();
      updi_mode_end = millis();
      break;
    case JTAG2::CMND_GET_SYNC:
    case JTAG2::CMND_GO:
      JTAG2::set_status(JTAG2::RSP_OK);
      break;
    case JTAG2::CMND_SET_DEVICE_DESCRIPTOR:
      JTAG2::set_device_descriptor();
      break;
    case JTAG2::CMND_READ_MEMORY:
      JTAG2::read_mem();
      break;
    case JTAG2::CMND_WRITE_MEMORY:
      JTAG2::write_mem();
      break;
    case JTAG2::CMND_XMEGA_ERASE:
      JTAG2::erase();
      break;
    default:
      JTAG2::set_status(JTAG2::RSP_FAILED);
  }
  
  JTAG2::answer();
  JTAG2::delay_exec();
}
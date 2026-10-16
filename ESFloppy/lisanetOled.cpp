// LisaNet OLED guard -- NOT part of upstream ESFloppy; written with an AI assistant. See lisanet.h.
//
// The OLED is driven over I2C from the SD task, the same task that feeds tracks to the floppy core, and the UI assumes a
// page goes out in ~3 ms (see sendOneOLEDPage()). On the LisaFPGA board a DEAD display (killed by a power fault) made
// those transfers hang, and the floppy core then got its tracks late: the Lisa saw read errors ("not in the standard
// Lisa Office System format"), intermittently, depending on how many screen updates landed during a read. Unplugging the
// display cured it. This guard makes a missing or failing display harmless: every OLED byte goes through guardByteCb,
// which drops all transfers if the display did not answer at start-up, or once any single transfer takes too long.
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "lisanet.h"

extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

#define OLED_I2C_ADDR      0x3C    // SH1106 (u8g2's 0x78, 8-bit)
#define OLED_XFER_LIMIT_US 5000    // a healthy transfer is well under 1 ms; a whole page ~3 ms

static bool oledOk = true;
static uint32_t xferStart = 0;

static uint8_t guardByteCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    if (!oledOk) return 1;                                   // pretend success: the UI carries on, nothing is sent
    if (msg == U8X8_MSG_BYTE_START_TRANSFER) xferStart = micros();
    uint8_t r = u8x8_byte_arduino_hw_i2c(u8x8, msg, arg_int, arg_ptr);
    if (msg == U8X8_MSG_BYTE_END_TRANSFER) {
        uint32_t dt = micros() - xferStart;
        if (dt > OLED_XFER_LIMIT_US) {
            oledOk = false;
            Serial.printf("[lisanet] OLED transfer took %lu us: display updates disabled so they can't stall the floppy\r\n",
                          (unsigned long)dt);
        }
    }
    return r;
}

void lisanetOledGuardInit() {
    Wire.setTimeOut(5);                                      // ms: bounds the probe if a dead display holds the bus
    Wire.beginTransmission(OLED_I2C_ADDR);
    uint8_t r = Wire.endTransmission();
    oledOk = (r == 0);
    Serial.printf("[lisanet] OLED %s (I2C result %u)\r\n",
                  oledOk ? "found" : "not answering: display updates disabled so they can't stall the floppy", r);
    u8g2_GetU8x8(OLED.getU8g2())->byte_cb = guardByteCb;     // every later OLED transfer goes through the guard
}

bool lisanetOledPresent() { return oledOk; }

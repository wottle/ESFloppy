// LisaNet: a copy of the LisaFPGA on-screen menu on the OLED -- NOT part of upstream ESFloppy; written with an AI
// assistant. See lisanet.h.
//
// While the FPGA's on-screen menu (or its picture-adjust mode) is up, the FPGA holds this ESP32's copies of the three
// buttons released -- so menu presses never reach ESFloppy's own screens -- and sends the menu's state on the link input
// (FPGA_4) as a frame in its control-frame format (docs/lisanet_link_protocol.md in the LisaFPGA repo, "Menu mirror
// frames"):
//   C3 3C 96 69 | 0x90 | seq | len | the FPGA's status bytes 0-13 | CRC-32 (zlib, over 0x90..payload, little-endian)
// at once on every change and every ~270 ms while it is up, then three times after it closes. The buttons still drive the
// FPGA's menu, so with this copy on the OLED the settings can be read and changed here -- handy when the monitor cannot
// show the picture (a resolution it does not support, say). The copy is dropped if frames stop for MENU_TIMEOUT_MS.
#include <Arduino.h>
#include <U8g2lib.h>
#include "uiState.h"
#include "profont6.h"
#include "lisanet.h"

extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

#define MENU_CMD        0x90
#define MENU_LEN        14
#define MENU_TIMEOUT_MS 800      // a frame comes at least every ~270 ms while the menu is up

static uint8_t st[MENU_LEN];     // the last good status block
static bool haveState = false;
static uint32_t lastFrameMs = 0;
static bool changed = false;

// ---- Frame receiver: fed every byte that arrives on the link, alongside the floppy remote's own A5 5A receiver ----
static uint32_t crcTab[256];
static bool crcReady = false;
static uint32_t crc32(const uint8_t* p, size_t n) {
    if (!crcReady) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            crcTab[i] = c;
        }
        crcReady = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = crcTab[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static const uint8_t MARK[4] = {0xC3, 0x3C, 0x96, 0x69};
static uint8_t fr[3 + 32 + 4];   // cmd, seq, len, payload, CRC
static size_t markHave = 0, have = 0, need = 0;

void lisanetMenuByte(uint8_t b) {
    if (markHave < 4) {                                  // looking for the marker
        if (b == MARK[markHave]) markHave++;
        else markHave = (b == MARK[0]) ? 1 : 0;
        have = 0;
        return;
    }
    fr[have++] = b;
    if (have == 3) {
        if (fr[2] > 32) { markHave = 0; return; }        // longer than any control frame: not one
        need = 3 + fr[2] + 4;
    }
    if (have < 3 || have < need) return;
    markHave = 0;                                        // a whole frame: check it, then look for the next marker
    size_t len = fr[2];
    uint32_t got = fr[3 + len] | (uint32_t)fr[4 + len] << 8 | (uint32_t)fr[5 + len] << 16 | (uint32_t)fr[6 + len] << 24;
    // Control frames from the WiFi board to the FPGA (STATUS, SET, ...) pass this way too: only 0x90 is ours
    if (fr[0] != MENU_CMD || len < MENU_LEN || crc32(fr, 3 + len) != got) return;
    if (!haveState || memcmp(st, fr + 3, MENU_LEN) != 0) changed = true;
    memcpy(st, fr + 3, MENU_LEN);
    haveState = true;
    lastFrameMs = millis();
}

// ---- The FPGA's status block (see HDMI_Interface.sv, "What the WiFi board reads back") ---------------------------
static uint16_t u16(int i) { return st[i] | (uint16_t)st[i + 1] << 8; }
static bool menuOpen()   { return st[13] & 0x80; }
static bool adjusting()  { return st[13] & 0x40; }
static bool adjustV()    { return st[13] & 0x20; }
static bool coarse()     { return st[13] & 0x10; }
static uint8_t menuSel() { return st[13] & 0x0F; }
static bool boardPresent() { return st[1] & 0x04; }

bool lisanetMenuShowing() {
    if (!haveState) return false;
    if (millis() - lastFrameMs > MENU_TIMEOUT_MS) {      // the FPGA stopped sending: drop the copy
        haveState = false;
        return false;
    }
    return menuOpen() || adjusting();
}

#define MENU_ITEMS 9
static const char* const LABELS[MENU_ITEMS] = {"ESP32 MODE", "RESOLUTION", "ADJUST HORIZ", "ADJUST VERT",
                                               "CPU SPEED", "MAX CONTRAST", "TV MENU", "SAVE SETTINGS", "EXIT"};

// The value column, worded as on the TV
static void itemValue(uint8_t item, char* out, size_t cap) {
    static const char* const speeds[4] = {"5MHZ", "10MHZ", "15MHZ", "18.75MHZ"};
    switch (item) {
    case 0: snprintf(out, cap, "%s", (st[1] & 0x08) ? ((st[1] & 0x10) ? "WIFI HDD" : "WIFI FDD")
                                     : (st[1] & 0x02) ? "DISABLED" : "NORMAL"); break;
    case 1: snprintf(out, cap, "%s", st[0] == 0 ? "1080P30" : st[0] == 1 ? "1080P60" : "1024X768"); break;
    case 2: snprintf(out, cap, "%u", u16(3)); break;
    case 3: if (u16(9) == 0) snprintf(out, cap, "NO ROOM"); else snprintf(out, cap, "%u", u16(7)); break;
    case 4: snprintf(out, cap, "%s", (st[1] & 0x80) ? speeds[st[2] & 3] : "SWITCHES"); break;
    case 5: snprintf(out, cap, "%s", (st[1] & 0x01) ? "ON" : "OFF"); break;
    case 6: snprintf(out, cap, "%s", (st[12] & 0x80) ? "OFF" : "ON"); break;
    case 7: snprintf(out, cap, "%s", (st[1] & 0x40) ? "SAVING" : (st[1] & 0x20) ? "SAVED" : "DEFAULT"); break;
    default: out[0] = 0;
    }
}

#define OLED_ROWS 8                                      // 64 px at MENU_ITEM_HEIGHT
static uint8_t menuTop = 0;                              // the first item shown when the list is longer than the screen

static void drawMenu() {
    uint8_t first = boardPresent() ? 0 : 1;              // ESP32 MODE is hidden without a WiFi board, as on the TV
    uint8_t rows = MENU_ITEMS - first;
    uint32_t y = 0;
    uint8_t top = first, last = MENU_ITEMS;
    if (rows < OLED_ROWS) {                              // a spare line: use it for a title
        OLED.drawStr((128 - OLED.getStrWidth("LISAFPGA MENU")) / 2, 0, "LISAFPGA MENU");
        OLED.drawHLine(0, MENU_ITEM_HEIGHT - 1, 128);
        y = MENU_ITEM_HEIGHT;
    } else if (rows > OLED_ROWS) {                       // too long: scroll so the highlighted row stays in view
        uint8_t sel = menuSel();
        if (menuTop < first) menuTop = first;
        if (sel < menuTop) menuTop = sel;
        if (sel >= menuTop + OLED_ROWS) menuTop = sel - OLED_ROWS + 1;
        if (menuTop > MENU_ITEMS - OLED_ROWS) menuTop = MENU_ITEMS - OLED_ROWS;
        top = menuTop;
        last = top + OLED_ROWS;
    }
    for (uint8_t item = top; item < last; item++, y += MENU_ITEM_HEIGHT) {
        char value[16];
        itemValue(item, value, sizeof value);
        bool sel = item == menuSel();
        if (sel) { OLED.setDrawColor(1); OLED.drawBox(0, y, 128, MENU_ITEM_HEIGHT); OLED.setDrawColor(0); }
        OLED.drawStr(2, y, LABELS[item]);
        OLED.drawStr(126 - OLED.getStrWidth(value), y, value);
        OLED.setDrawColor(1);
    }
}

static void drawAdjust() {
    const char* title = adjustV() ? "ADJUST VERT" : "ADJUST HORIZ";
    OLED.drawStr((128 - OLED.getStrWidth(title)) / 2, 0, title);
    OLED.drawHLine(0, MENU_ITEM_HEIGHT - 1, 128);
    char num[8], lim[24];
    snprintf(num, sizeof num, "%u", adjustV() ? u16(7) : u16(3));
    snprintf(lim, sizeof lim, "of %u%s", adjustV() ? u16(9) : u16(5), coarse() ? "   FAST" : "");
    OLED.setFont(u8g2_font_profont22_tn);
    OLED.drawStr((128 - OLED.getStrWidth(num)) / 2, 11, num);
    OLED.setFont(u8g2_font_profont6_tr);
    OLED.drawStr((128 - OLED.getStrWidth(lim)) / 2, 11 + 24, lim);
    OLED.drawStr(0, 64 - 2 * MENU_ITEM_HEIGHT, "LEFT/RIGHT: move");
    OLED.drawStr(0, 64 - MENU_ITEM_HEIGHT, "OK: save");
}

// ---- Splash: shown for SPLASH_MS when the menu opens, or until the first button press moves or changes something ----
#define SPLASH_MS 1000
static uint32_t splashStart = 0;
static bool splashOn = false;
static uint8_t splashMenuByte = 0;

// A settings gear centred on (cx, cy): a disc with eight square teeth and a hole in the middle
static void drawGear(int cx, int cy) {
    const float rIn = 9.0f, rOut = 15.5f, half = 3.2f;   // teeth run from rIn to rOut, 2 * half wide
    for (int k = 0; k < 8; k++) {
        float a = k * (float)M_PI / 4, c = cosf(a), s = sinf(a);
        // The tooth's four corners: across the tooth is (-s, c)
        int x0 = lroundf(cx + c * rIn - s * half),  y0 = lroundf(cy + s * rIn + c * half);
        int x1 = lroundf(cx + c * rIn + s * half),  y1 = lroundf(cy + s * rIn - c * half);
        int x2 = lroundf(cx + c * rOut + s * half), y2 = lroundf(cy + s * rOut - c * half);
        int x3 = lroundf(cx + c * rOut - s * half), y3 = lroundf(cy + s * rOut + c * half);
        OLED.drawTriangle(x0, y0, x1, y1, x2, y2);
        OLED.drawTriangle(x0, y0, x2, y2, x3, y3);
    }
    OLED.drawDisc(cx, cy, 11);
    OLED.setDrawColor(0);
    OLED.drawDisc(cx, cy, 4);                            // the axle hole
    OLED.setDrawColor(1);
}

static void drawSplash() {
    drawGear(24, 20);
    OLED.setFont(u8g2_font_profont17_tr);
    OLED.drawStr(50, 6, "Lisa");
    OLED.drawStr(50, 21, "FPGA");
    OLED.setFont(u8g2_font_profont6_tr);
    OLED.drawHLine(0, 41, 128);
    OLED.drawStr((128 - OLED.getStrWidth("SETTINGS MENU")) / 2, 44, "SETTINGS MENU");
    OLED.drawStr((128 - OLED.getStrWidth("L/R: move  OK: choose")) / 2, 55, "L/R: move  OK: choose");
}

// Redraws the frame buffer when the state changed; returns true if it did (the caller sends it to the OLED)
bool lisanetMenuDraw() {
    if (splashOn && (millis() - splashStart >= SPLASH_MS || st[13] != splashMenuByte || !menuOpen())) {
        splashOn = false;                                // time's up, or a press already did something
        changed = true;
    }
    if (!changed) return false;
    changed = false;
    OLED.clearBuffer();
    OLED.setFont(u8g2_font_profont6_tr);
    OLED.setDrawColor(1);
    if (splashOn) drawSplash();
    else if (adjusting()) drawAdjust();
    else drawMenu();
    return true;
}

// Called when the copy appears, so it is drawn even if the state matches the last copy shown. Opening the menu itself
// (not adjust mode) starts with the splash.
void lisanetMenuForceRedraw() {
    changed = true;
    splashOn = menuOpen() && !adjusting();
    splashStart = millis();
    splashMenuByte = st[13];
}

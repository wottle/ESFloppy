// LisaNet: WiFi setup help on the OLED -- NOT part of upstream ESFloppy; written with an AI assistant. See lisanet.h.
//
// While the LisaFPGA board's WiFi board (the XIAO on J21) has its setup network up, it sends WIFI_NOTICE (0x54) over the
// floppy remote's link every couple of seconds (docs/lisanet_link_protocol.md in the LisaFPGA repo):
//   u8 state (1 = the setup network is up, 2 = it just joined a network), then four u8-length strings: network name,
//   network password (empty when state 2), address, and the web page's host name.
// The OLED then alternates a QR code that joins the setup network with written instructions, and after a join shows
// where to find the web page. Any button hides it (until the state changes, or the notices stop for a while and
// start again); it also goes away by itself NOTICE_TIMEOUT_MS after the last notice.
#include <Arduino.h>
#include <U8g2lib.h>
#include <qrcode.h>
#include "uiState.h"
#include "profont6.h"
#include "lisanet.h"

extern U8G2_SH1106_128X64_NONAME_F_HW_I2C OLED;

#define NOTICE_TIMEOUT_MS 6000   // the WiFi board repeats a notice every ~2 s
#define QR_PAGE_MS        8000   // the setup notice alternates: QR code ...
#define TEXT_PAGE_MS      6000   // ... then the written instructions
#define QR_MAX_VERSION    3      // 29 x 29 modules: at 2 px each, with its light margin, the panel's full height

enum { NOTICE_NONE = 0, NOTICE_SETUP = 1, NOTICE_JOINED = 2 };

static uint8_t state = NOTICE_NONE;
static char ssid[33], pass[65], addr[16], host[32];
static uint32_t lastNoticeMs = 0;
static uint8_t dismissedState = NOTICE_NONE;   // hidden by a button press until the state changes
static bool changed = false;

// ---- The QR code: built once per network name and password -------------------------------------------------------
static uint8_t qrSize = 0;                      // modules per side; 0 = none (the text page is shown alone)
static uint8_t qrBits[29][29];
static char qrFor[33 + 65];

static void qrCopy(esp_qrcode_handle_t q) {
    int n = esp_qrcode_get_size(q);
    if (n > 29) return;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) qrBits[y][x] = esp_qrcode_get_module(q, x, y);
    qrSize = n;
}

// The WIFI: QR format escapes \ ; , : and " with a backslash
static void appendEscaped(char* out, size_t cap, const char* s) {
    size_t o = strlen(out);
    for (; *s && o + 2 < cap; s++) {
        if (strchr("\\;,:\"", *s)) out[o++] = '\\';
        out[o++] = *s;
    }
    out[o] = 0;
}

static void buildQr() {
    char key[sizeof qrFor];
    snprintf(key, sizeof key, "%s\n%s", ssid, pass);
    if (!strcmp(key, qrFor)) return;
    strcpy(qrFor, key);
    char text[200] = "WIFI:T:";
    strcat(text, pass[0] ? "WPA" : "nopass");
    strcat(text, ";S:");
    appendEscaped(text, sizeof text, ssid);
    if (pass[0]) { strcat(text, ";P:"); appendEscaped(text, sizeof text, pass); }
    strcat(text, ";;");
    qrSize = 0;
    esp_qrcode_config_t cfg = {qrCopy, QR_MAX_VERSION, ESP_QRCODE_ECC_LOW};
    if (esp_qrcode_generate(&cfg, text) != ESP_OK) qrSize = 0;   // too long for the panel: text only
    Serial.printf("[lisanet] WiFi setup QR code: %s\r\n", qrSize ? "ready" : "does not fit, text only");
}

// ---- Notices from the WiFi board (lisanetDrive.cpp) ------------------------------------------------------------------
static bool getStr(const uint8_t*& p, const uint8_t* end, char* out, size_t cap) {
    if (p >= end || p + 1 + *p > end) return false;
    size_t n = *p++;
    size_t k = n < cap - 1 ? n : cap - 1;
    memcpy(out, p, k);
    out[k] = 0;
    p += n;
    return true;
}

void lisanetWifiNotice(const uint8_t* p, size_t n) {
    const uint8_t* end = p + n;
    if (n < 1 || (p[0] != NOTICE_SETUP && p[0] != NOTICE_JOINED)) return;
    uint8_t s = *p++;
    char nSsid[sizeof ssid], nPass[sizeof pass], nAddr[sizeof addr], nHost[sizeof host];
    if (!getStr(p, end, nSsid, sizeof nSsid) || !getStr(p, end, nPass, sizeof nPass) ||
        !getStr(p, end, nAddr, sizeof nAddr) || !getStr(p, end, nHost, sizeof nHost)) return;
    uint32_t now = millis();
    // A lapse in the notices starts a new episode: show it again even if it was hidden
    if (state == NOTICE_NONE || now - lastNoticeMs > NOTICE_TIMEOUT_MS) dismissedState = NOTICE_NONE;
    if (s != state || strcmp(nSsid, ssid) || strcmp(nPass, pass) || strcmp(nAddr, addr) || strcmp(nHost, host)) {
        if (s != state) { dismissedState = NOTICE_NONE; Serial.printf("[lisanet] WiFi notice: %s\r\n", s == NOTICE_SETUP ? "setup network up" : "joined"); }
        changed = true;
    }
    state = s;
    strcpy(ssid, nSsid); strcpy(pass, nPass); strcpy(addr, nAddr); strcpy(host, nHost);
    lastNoticeMs = now;
}

bool lisanetWifiShowing() {
    if (state == NOTICE_NONE) return false;
    if (millis() - lastNoticeMs > NOTICE_TIMEOUT_MS) { state = NOTICE_NONE; return false; }
    return state != dismissedState;
}

void lisanetWifiDismiss() {
    dismissedState = state;
    Serial.println("[lisanet] WiFi notice hidden by a button press");
}

// ---- Drawing ---------------------------------------------------------------------------------------------------------
static uint32_t pageStart = 0;
static bool qrPage = true;

static void drawQrPage() {
    // Dark modules on a lit square: the light margin around the code is the 3 px border of the square
    OLED.drawBox(0, 0, 64, 64);
    OLED.setDrawColor(0);
    int off = (64 - qrSize * 2) / 2;
    for (int y = 0; y < qrSize; y++)
        for (int x = 0; x < qrSize; x++)
            if (qrBits[y][x]) OLED.drawBox(off + x * 2, off + y * 2, 2, 2);
    OLED.setDrawColor(1);
    static const char* const lines[] = {"WIFI SETUP", "", "Scan with", "your phone", "camera to", "join, then", "follow the",
                                        "page."};
    for (int i = 0; i < 8; i++) OLED.drawStr(67, i * MENU_ITEM_HEIGHT, lines[i]);
    OLED.drawHLine(67, MENU_ITEM_HEIGHT - 1, 61);
}

static void drawTitle(const char* t) {
    OLED.drawStr((128 - OLED.getStrWidth(t)) / 2, 0, t);
    OLED.drawHLine(0, MENU_ITEM_HEIGHT - 1, 128);
}

static void drawTextPage() {
    char line[40];
    drawTitle("WIFI SETUP");
    OLED.drawStr(0, 8, "Join this network:");
    snprintf(line, sizeof line, " %s", ssid);
    OLED.drawStr(0, 16, line);
    if (pass[0]) {
        OLED.drawStr(0, 24, "Password:");
        snprintf(line, sizeof line, " %s", pass);
        OLED.drawStr(0, 32, line);
    }
    OLED.drawStr(0, 40, "The setup page opens,");
    snprintf(line, sizeof line, "or go to %s", addr);
    OLED.drawStr(0, 48, line);
    OLED.drawStr(0, 56, "Any button: hide");
}

static void drawJoinedPage() {
    char line[40];
    drawTitle("WIFI CONNECTED");
    OLED.drawStr(0, 8, "Network:");
    snprintf(line, sizeof line, " %s", ssid);
    OLED.drawStr(0, 16, line);
    OLED.drawStr(0, 24, "LisaNet web page:");
    snprintf(line, sizeof line, " http://%s", host);
    OLED.drawStr(0, 32, line);
    snprintf(line, sizeof line, " or http://%s", addr);
    OLED.drawStr(0, 40, line);
    OLED.drawStr(0, 56, "Any button: hide");
}

// Redraws the frame buffer when something changed or the page flipped; returns true if it did
bool lisanetWifiDraw() {
    uint32_t now = millis();
    if (state == NOTICE_SETUP) {
        buildQr();
        if (now - pageStart >= (qrPage ? QR_PAGE_MS : TEXT_PAGE_MS)) {
            qrPage = !qrPage;
            pageStart = now;
            changed = true;
        }
    }
    if (!changed) return false;
    changed = false;
    OLED.clearBuffer();
    OLED.setFont(u8g2_font_profont6_tr);
    OLED.setDrawColor(1);
    if (state == NOTICE_JOINED) drawJoinedPage();
    else if (qrPage && qrSize) drawQrPage();
    else drawTextPage();
    return true;
}

// Called when the notice appears: drawn from the start, QR code first
void lisanetWifiForceRedraw() {
    changed = true;
    qrPage = true;
    pageStart = millis();
}

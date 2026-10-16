// LisaNet floppy remote control -- NOT part of upstream ESFloppy; written with an AI assistant. See lisanet.h.
//
// In NORMAL mode the LisaFPGA board connects ESFloppy's link lines (FPGA_4 in, FPGA_5 out) to the WiFi board on J21, so
// while ESFloppy is the Lisa's floppy drive the WiFi board's web page can see what is inserted, insert an image and eject.
// Same frames as the SD server (A5 5A, CRC-32; docs/lisanet_link_protocol.md in the LisaFPGA repo), but only these
// commands, which the SD server does not have -- so the WiFi board can tell a drive from a server, and nothing here
// writes to the card:
//   0x50 DRIVE_STATUS  -> u8 version, u8 emulMode, u8 flags (bit0 Twiggy, bit1 ESFloppy's own menu is open),
//                         then for drive 0 and drive 1: u8 flags (bit0 exists in this mode, bit1 inserted,
//                         bit2 eject pending), u8 name_len, name; then the firmware string
//   0x51 DRIVE_LIST    u16 start, path -> u16 next (0xFFFF = end), entries: u8 flags (bit0 directory), u64 size,
//                         u8 name_len, name. Only folders and images this drive mode accepts, as the OLED picker shows.
//   0x52 DRIVE_INSERT  u8 drive, path -> status
//   0x53 DRIVE_EJECT   u8 drive, u8 force -> status. Sony drives have no eject button, so they always force-eject
//                         (as a long SEL press does); a Twiggy can be asked to eject normally.
// Everything runs in the SD task's idle loop, between the floppy core's track requests, exactly where the OLED UI and
// its file picker run -- so an insert here is the same operation as one picked on the OLED.
#include <Arduino.h>
#include <SdFat.h>
#include "types.h"
#include "diskLib.h"
#include "uiState.h"
#include "fwVersion.h"
#include "lisanet.h"

extern SdFat32 SDCard;
extern File32 disk[2];
extern DiskImageMetadata* diskMetadataPointers[2];
extern char driveFilenames[2][1024];
extern ConfigSettings configSettings;
extern TrackParams* sonyUiTrackParams;
extern TrackParams* twiggyUiTrackParams[2];
extern Screen* currentScreen;
extern Screen statusScreen;

enum : uint8_t { ST_OK = 0, ST_NOT_FOUND = 1, ST_EXISTS = 2, ST_IO = 3, ST_BAD_REQUEST = 4, ST_UNKNOWN_CMD = 6,
                 ST_NO_CONTIGUOUS = 8, ST_UI_BUSY = 9, ST_BAD_IMAGE = 10, ST_IN_USE = 11 };
enum : uint8_t { CMD_DRIVE_STATUS = 0x50, CMD_DRIVE_LIST = 0x51, CMD_DRIVE_INSERT = 0x52, CMD_DRIVE_EJECT = 0x53 };

#define DRV_MAX_REQ  600
#define DRV_MAX_RESP 2400

static HardwareSerial Link(1);
static bool linkUp = false;
static uint32_t rxHighSince = 0;

static uint32_t crcTable[256];
static void crcInit() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crcTable[i] = c;
    }
}
static uint32_t crcUpdate(uint32_t c, const uint8_t* p, size_t n) {
    while (n--) c = crcTable[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return c;
}
static uint16_t rd16(const uint8_t* p) { return p[0] | (uint16_t)p[1] << 8; }

static uint8_t resp[DRV_MAX_RESP];
static size_t respLen;
static void begin(uint8_t st) { resp[0] = st; respLen = 1; }
static void put8(uint8_t v) { if (respLen < DRV_MAX_RESP) resp[respLen++] = v; }
static void put16(uint16_t v) { put8(v); put8(v >> 8); }
static void put64(uint64_t v) { for (int i = 0; i < 8; i++) { put8(v); v >>= 8; } }
static void putStr(const char* s, size_t max) {
    size_t n = strnlen(s, max);
    put8(n);
    for (size_t i = 0; i < n; i++) put8(s[i]);
}

static bool twiggyMode() { return configSettings.emulMode == ModeTwiggy; }
static bool driveExists(uint8_t d) { return d == 1 || (d == 0 && twiggyMode()); }
static TrackParams* trackParams(uint8_t d) { return twiggyMode() ? twiggyUiTrackParams[d] : sonyUiTrackParams; }

static bool getPath(const uint8_t* p, size_t n, char* out, size_t cap) {
    if (n >= cap) return false;
    memcpy(out, p, n);
    out[n] = 0;
    if (n == 0) strcpy(out, "/");
    return true;
}

// Whether the OLED picker would offer this file in the current drive mode. Mirrors the checks in
// assembleDirectoryContents() (uiFilePicker.cpp): extension, hidden name, and the DC42 header or raw size.
static bool imageCompatible(File32& f, const char* name) {
    const char* ext = strrchr(name, '.');
    if (!ext || (strcasecmp(ext, ".dc42") && strcasecmp(ext, ".image") && strcasecmp(ext, ".img"))) return false;
    if (name[0] == '.') return false;
    uint8_t sector[512];
    if (!f.seekSet(0) || f.read(sector, sizeof sector) < (int)sizeof sector) return false;
    DC42Header h;
    memcpy(&h, sector, sizeof h);
    uint32_t dataSize = __builtin_bswap32(h.dataSize), tagSize = __builtin_bswap32(h.tagSize);
    bool twiggy = twiggyMode();
    if (__builtin_bswap16(h.dc42Magic) == 0x0100) {
        if (f.fileSize() < sizeof(DC42Header) + dataSize + tagSize) return false;
        if (h.diskEncoding == DC42_400K_ENCODING && dataSize == DATA_SIZE_400K && !twiggy)
            return tagSize == TAG_SIZE_400K || tagSize == 0;
        if (h.diskEncoding == DC42_800K_ENCODING && dataSize == DATA_SIZE_800K && !twiggy)
            return tagSize == TAG_SIZE_800K || tagSize == 0;
        if (h.diskEncoding == DC42_TWIGGY_ENCODING && dataSize == DATA_SIZE_TWIGGY && twiggy)
            return tagSize == TAG_SIZE_TWIGGY || tagSize == 0;
        return false;
    }
    uint64_t sz = f.fileSize();
    return twiggy ? sz == DATA_SIZE_TWIGGY : (sz == DATA_SIZE_400K || sz == DATA_SIZE_800K);
}

static void handle(uint8_t cmd, const uint8_t* p, size_t n) {
    char path[256];
    switch (cmd) {
    case CMD_DRIVE_STATUS: {
        begin(ST_OK);
        put8(1);
        put8(configSettings.emulMode);
        put8((twiggyMode() ? 1 : 0) | (currentScreen != &statusScreen ? 2 : 0));
        for (uint8_t d = 0; d < 2; d++) {
            bool in = driveExists(d) && diskMetadataPointers[d]->diskInserted;
            bool ej = driveExists(d) && trackParams(d)->ejectRequested != EjectNone;
            put8((driveExists(d) ? 1 : 0) | (in ? 2 : 0) | (ej ? 4 : 0));
            putStr(in ? driveFilenames[d] : "", 255);
        }
        putStr("ESFloppy " FIRMWARE_VERSION " + LisaNet floppy remote 0.1", 64);
        break;
    }
    case CMD_DRIVE_LIST: {
        if (n < 2 || !getPath(p + 2, n - 2, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        uint16_t start = rd16(p), index = 0;
        File32 dir, e;
        if (!dir.open(path, O_RDONLY) || !dir.isDir()) { begin(ST_NOT_FOUND); return; }
        begin(ST_OK);
        size_t nextAt = respLen;
        put16(0xFFFF);
        char name[256];
        while (e.openNext(&dir, O_RDONLY)) {
            e.getName(name, sizeof name);
            bool isDir = e.isDir();
            bool show = name[0] && name[0] != '.' && (isDir || imageCompatible(e, name));
            uint64_t size = e.fileSize();
            e.close();
            if (!show) continue;
            if (index++ < start) continue;
            size_t nl = strlen(name);
            if (respLen + 1 + 8 + 1 + nl > DRV_MAX_RESP) {               // full: tell the client where to resume
                resp[nextAt] = (index - 1) & 0xFF; resp[nextAt + 1] = (index - 1) >> 8;
                break;
            }
            put8(isDir ? 1 : 0); put64(isDir ? 0 : size); putStr(name, 255);
        }
        dir.close();
        break;
    }
    case CMD_DRIVE_INSERT: {
        if (n < 2 || !getPath(p + 1, n - 1, path, sizeof path)) { begin(ST_BAD_REQUEST); return; }
        uint8_t d = p[0];
        if (!driveExists(d)) { begin(ST_BAD_REQUEST); return; }
        if (currentScreen != &statusScreen) { begin(ST_UI_BUSY); return; }       // the OLED picker may be choosing
        if (diskMetadataPointers[d]->diskInserted || trackParams(d)->ejectRequested != EjectNone) { begin(ST_EXISTS); return; }
        if (diskMetadataPointers[1 - d]->diskInserted && strcmp(path, driveFilenames[1 - d]) == 0) { begin(ST_IN_USE); return; }
        {
            File32 f;
            if (!f.open(path, O_RDONLY) || f.isDir()) { begin(ST_NOT_FOUND); return; }
            const char* base = strrchr(path, '/');
            bool ok = imageCompatible(f, base ? base + 1 : path);
            f.close();
            if (!ok) { begin(ST_BAD_IMAGE); return; }
        }
        OpenResult r = openImage(path, &disk[d], diskMetadataPointers[d]);   // as the OLED picker does
        if (r == ResultSuccess) {
            diskMetadataPointers[d]->diskInserted = true;
            strncpy(driveFilenames[d], path, sizeof(driveFilenames[d]));
            Serial.printf("[lisanet] remote: inserted %s in drive %u\r\n", path, (unsigned)d);
            begin(ST_OK);
        } else {
            Serial.printf("[lisanet] remote: could not insert %s (open result %d)\r\n", path, (int)r);
            begin(r == ResultNotContiguous ? ST_NO_CONTIGUOUS : r == ResultFailedOpen ? ST_IO : ST_BAD_IMAGE);
        }
        break;
    }
    case CMD_DRIVE_EJECT: {
        if (n != 2 || !driveExists(p[0])) { begin(ST_BAD_REQUEST); return; }
        uint8_t d = p[0];
        if (!diskMetadataPointers[d]->diskInserted) { begin(ST_NOT_FOUND); return; }
        trackParams(d)->ejectRequested = (p[1] || !twiggyMode()) ? EjectForce : EjectNormal;
        Serial.printf("[lisanet] remote: %s eject of drive %u\r\n", trackParams(d)->ejectRequested == EjectForce ? "force" : "normal",
                      (unsigned)d);
        begin(ST_OK);
        break;
    }
    default:
        begin(ST_UNKNOWN_CMD);     // never sent back: see frame() -- a drive stays silent to the SD server's commands
    }
}

// Frame receiver (as in sdServer.cpp)
static uint8_t rx[6 + DRV_MAX_REQ + 4];
static size_t have = 0, need = 0;
static uint32_t lastByte = 0;

static void frame() {
    uint16_t len = rd16(rx + 4);
    uint32_t got = rx[6 + len] | (uint32_t)rx[7 + len] << 8 | (uint32_t)rx[8 + len] << 16 | (uint32_t)rx[9 + len] << 24;
    if ((crcUpdate(0xFFFFFFFFu, rx + 2, 4 + len) ^ 0xFFFFFFFFu) != got) return;
    uint8_t cmd = rx[2], seq = rx[3];
    // Only the drive commands are answered. The SD server's commands (HELLO, LIST, ...) get no reply, so the WiFi board
    // still sees "no SD server" in NORMAL mode and never mistakes the drive for one.
    if (cmd < CMD_DRIVE_STATUS || cmd > CMD_DRIVE_EJECT) return;
    handle(cmd, rx + 6, len);
    uint8_t hdr[6] = {0xA5, 0x5A, (uint8_t)(cmd | 0x80), seq, (uint8_t)respLen, (uint8_t)(respLen >> 8)};
    uint32_t c = crcUpdate(crcUpdate(0xFFFFFFFFu, hdr + 2, 4), resp, respLen) ^ 0xFFFFFFFFu;
    uint8_t tail[4] = {(uint8_t)c, (uint8_t)(c >> 8), (uint8_t)(c >> 16), (uint8_t)(c >> 24)};
    Link.write(hdr, 6);
    Link.write(resp, respLen);
    Link.write(tail, 4);
}

void lisanetDrivePoll() {
    if (!linkUp) {
        // Start the UART only once the link line has idled high for 200 ms: an older FPGA build (or no WiFi board) holds
        // it low, and then this ESP32's TX pin is never driven at all.
        if (digitalRead(LISANET_LINK_RX_PIN) == LOW) { rxHighSince = 0; return; }
        if (!rxHighSince) { rxHighSince = millis() | 1; return; }
        if (millis() - rxHighSince < 200) return;
        crcInit();
        Link.setRxBufferSize(1024);
        Link.setTxBufferSize(DRV_MAX_RESP + 16);
        Link.begin(LISANET_LINK_BAUD, SERIAL_8N1, LISANET_LINK_RX_PIN, LISANET_LINK_TX_PIN);
        linkUp = true;
        Serial.println("[lisanet] floppy remote control: link up");
        return;
    }
    if (have && millis() - lastByte > 100) have = 0;          // abandon a frame that stopped arriving
    while (Link.available()) {
        uint8_t b = Link.read();
        lastByte = millis();
        if (have == 0 && b != 0xA5) continue;
        if (have == 1 && b != 0x5A) { have = (b == 0xA5) ? 1 : 0; continue; }
        rx[have++] = b;
        if (have == 6) {
            uint16_t len = rd16(rx + 4);
            if (len > DRV_MAX_REQ) { have = 0; continue; }      // longer than any drive command: not for us
            need = 6 + len + 4;
        }
        if (have >= 6 && have == need) { frame(); have = 0; }
    }
}

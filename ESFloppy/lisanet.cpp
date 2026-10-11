// LisaNet MODE-line helpers -- NOT part of upstream ESFloppy; written with an AI assistant. See lisanet.h.
#include "lisanet.h"

bool lisanetModeStableAt(bool level) {
    static uint32_t since = 0;
    static bool primed = false;
    uint32_t now = millis();
    if (!primed) { since = now; primed = true; }
    if (digitalRead(LISANET_MODE_PIN) != level) { since = now; return false; }
    return now - since >= LISANET_MODE_STABLE_MS;
}

bool lisanetModeRequestedAtBoot() {
    pinMode(LISANET_MODE_PIN, INPUT);
    // Watch for up to twice the stability window: WIFI only if MODE is high for a full window
    uint32_t start = millis(), since = millis();
    while (millis() - start < 2 * LISANET_MODE_STABLE_MS) {
        if (digitalRead(LISANET_MODE_PIN) == LOW) return false;
        if (millis() - since >= LISANET_MODE_STABLE_MS) return true;
        delay(5);
    }
    return true;
}

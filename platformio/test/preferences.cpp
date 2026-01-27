#include <Preferences.h>

Preferences prefs;

// ----------------------
// Keys as required
// ----------------------
static const char *PREF_NAMESPACE   = "mpu";
static const char *PREF_KEY_OFFS    = "offs";     // raw bytes
static const char *PREF_KEY_SECTIONS= "sections"; // byte array
static const char *PREF_KEY_SENS    = "sens";     // int
static const char *PREF_KEY_DIR     = "dir";      // bool
static const char *PREF_KEY_LEGATO  = "legato";   // bool


// ------------------------------------------------------------
// Example structure for PREF_KEY_OFFS (MPU offsets)
// Replace with your real struct if needed.
// ------------------------------------------------------------
struct MPUOffsets {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

MPUOffsets testOffsets = { 100, -120, 250, 10, -5, 30 };


// ------------------------------------------------------------
// Utility: print raw bytes in hex
// ------------------------------------------------------------
void printHex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++)
    Serial.printf("%02X ", data[i]);
  Serial.println();
}


// ------------------------------------------------------------
// Read & Print all values
// ------------------------------------------------------------
void readAll() {
  Serial.println("\n=== Reading all NVS values ===");

  // ---- OFFS (struct as bytes) ----
  size_t offs_len = prefs.getBytesLength(PREF_KEY_OFFS);
  Serial.printf("OFFS bytes length: %u\n", (unsigned)offs_len);
  if (offs_len > 0) {
    uint8_t buf[64];
    size_t n = prefs.getBytes(PREF_KEY_OFFS, buf, sizeof(buf));
    Serial.print("OFFS data: ");
    printHex(buf, n);
  } else {
    Serial.println("OFFS not set");
  }

  // ---- SECTIONS (byte array) ----
  size_t sec_len = prefs.getBytesLength(PREF_KEY_SECTIONS);
  Serial.printf("SECTIONS bytes length: %u\n", (unsigned)sec_len);
  if (sec_len > 0) {
    uint8_t buf[64];
    size_t n = prefs.getBytes(PREF_KEY_SECTIONS, buf, sizeof(buf));
    Serial.print("SECTIONS data: ");
    printHex(buf, n);
  } else {
    Serial.println("SECTIONS not set");
  }

  // ---- SENS (int) ----
  int sens = prefs.getInt(PREF_KEY_SENS, -1);
  Serial.printf("SENS value: %d\n", sens);

  // ---- DIR (bool) ----
  bool dir = prefs.getBool(PREF_KEY_DIR, false);
  Serial.printf("DIR (bool): %s\n", dir ? "true" : "false");

  // ---- LEGATO (bool) ----
  bool leg = prefs.getBool(PREF_KEY_LEGATO, false);
  Serial.printf("LEGATO (bool): %s\n", leg ? "true" : "false");

  Serial.println("==============================\n");
}


// ------------------------------------------------------------
// Write demo values
// ------------------------------------------------------------
void writeDemoValues() {
  Serial.println("\n=== Writing demo values to NVS ===");

  // Write struct (OFFS)
  prefs.putBytes(PREF_KEY_OFFS, &testOffsets, sizeof(testOffsets));
  Serial.println("Wrote OFFS struct");

  // Example SECTIONS array
  uint8_t sections[6] = {1, 3, 5, 7, 9, 11};
  prefs.putBytes(PREF_KEY_SECTIONS, sections, sizeof(sections));
  Serial.println("Wrote SECTIONS array");

  // Write SENS int
  prefs.putInt(PREF_KEY_SENS, 1234);
  Serial.println("Wrote SENS = 1234");

  // Write DIR bool
  prefs.putBool(PREF_KEY_DIR, true);
  Serial.println("Wrote DIR = true");

  // Write LEGATO bool
  prefs.putBool(PREF_KEY_LEGATO, false);
  Serial.println("Wrote LEGATO = false");

  Serial.println("==============================\n");
}


// ------------------------------------------------------------
// Erase individual keys
// ------------------------------------------------------------
void eraseKeys() {
  Serial.println("Erasing individual keys...");
  prefs.remove(PREF_KEY_OFFS);
  prefs.remove(PREF_KEY_SECTIONS);
  prefs.remove(PREF_KEY_SENS);
  prefs.remove(PREF_KEY_DIR);
  prefs.remove(PREF_KEY_LEGATO);
  Serial.println("Keys erased.\n");
}


// ------------------------------------------------------------
// Clear whole namespace
// ------------------------------------------------------------
void clearNamespace() {
  Serial.println("Clearing entire namespace...");
  prefs.clear();
  Serial.println("Namespace cleared.\n");
}


// ------------------------------------------------------------
// Setup & menu
// ------------------------------------------------------------
void printMenu() {
  Serial.println("\n=== MENU ===");
  Serial.println("r = read all");
  Serial.println("w = write demo values");
  Serial.println("e = erase keys");
  Serial.println("c = clear namespace");
  Serial.println("x = close and reopen prefs");
  Serial.println("==============\n");
}


void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n*** ESP32 NVS / Preferences Test ***");

  prefs.begin(PREF_NAMESPACE, false);  // RW mode
  Serial.println("Opened NVS namespace: mpu\n");

  readAll();
  printMenu();
}


void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'r': readAll(); break;
      case 'w': writeDemoValues(); break;
      case 'e': eraseKeys(); break;
      case 'c': clearNamespace(); break;

      case 'x':
        Serial.println("Closing prefs...");
        prefs.end();
        delay(200);
        Serial.println("Reopening prefs...");
        prefs.begin(PREF_NAMESPACE, false);
        break;

      case '?':
      default:
        printMenu();
        break;
    }
  }
}

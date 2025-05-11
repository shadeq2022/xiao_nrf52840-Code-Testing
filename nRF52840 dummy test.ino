// BLE peripheral untuk nRF52840 yang mengirim data tiap 2 detik
// Data terdiri dari status (0/1) dan timestamp (detik)
// Gunakan library Adafruit Bluefruit untuk komunikasi BLE
// Inisialisasi BLE service, characteristic, dan advertising
// Setiap 2 detik, kirim notifikasi BLE dan tampilkan data ke Serial Monitor

#include <bluefruit.h>

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

BLEService myService = BLEService(SERVICE_UUID);
BLECharacteristic myCharacteristic = BLECharacteristic(CHARACTERISTIC_UUID);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial
  Serial.println("nRF52840 BLE Peripheral");

  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Max power
  Bluefruit.setName("nRF52840");

  myService.begin();
  myCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  myCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  myCharacteristic.setFixedLen(5); // 1 byte for status + 4 bytes for timestamp
  myCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(myService);
  Bluefruit.Advertising.start();
  Serial.println("Advertising started...");
}

void loop() {
  static uint32_t lastMillis = 0;
  static bool state = false;

  if (millis() - lastMillis > 2000) {  // Send every 1 second
    lastMillis = millis();
    
    // Prepare data: 1 byte for status + 4 bytes for timestamp
    uint8_t data[5];
    data[0] = state ? 1 : 0; // Status (0 or 1)
    
    // Get current timestamp (in seconds since startup)
    uint32_t timestamp = millis() / 2000; // Convert to seconds
    memcpy(&data[1], &timestamp, 4); // Copy timestamp to data array

    // Send data via BLE
    myCharacteristic.notify(data, 5);
    
    // Print data to Serial Monitor for debugging
    Serial.print("Data sent - Status: ");
    Serial.print(data[0]);
    Serial.print(", Timestamp: ");
    Serial.println(timestamp);

    state = !state;  // Toggle state for next send
  }
}
#include <bluefruit.h>
#include <PDM.h>
#include <snore-detection-v4-fix_inferencing.h>

#define ENABLE_SERIAL false // Set false untuk versi baterai
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

BLEService myService = BLEService(SERVICE_UUID);
BLECharacteristic myCharacteristic = BLECharacteristic(CHARACTERISTIC_UUID);

/* Audio buffer and inference variables */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

void setup() {
  if (ENABLE_SERIAL) { // cek apakah kabel usb terhubung untuk serial debug pakai USB dan bisa buka serial monitor
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
  }

  // Initialize BLE
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("SnoreDetector");

  myService.begin();
  myCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  myCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  myCharacteristic.setFixedLen(5); // 1 byte status + 4 bytes timestamp
  myCharacteristic.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(myService);
  Bluefruit.Advertising.start();
  Serial.println("BLE Advertising started");

  // Initialize Edge Impulse
  Serial.println("Initializing Edge Impulse model...");
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / 
                                      sizeof(ei_classifier_inferencing_categories[0]));

  run_classifier_init();
  if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
    ei_printf("ERR: Failed to initialize microphone!\r\n");
    return;
  }
}

void loop() {
  // Process Edge Impulse inference
  bool m = microphone_inference_record();
  if (!m) {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, false);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  // Kirim hasil secara periodik
  if (++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW) {

    // 1. Tampilkan informasi waktu proses dari model
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    // 2. Cari label dengan skor kepercayaan tertinggi dan tampilkan semua hasil debug
    float max_confidence = 0.0;
    int predicted_index = -1;

    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      ei_printf("    %s: %.5f\n", result.classification[i].label, result.classification[i].value);
      if (result.classification[i].value > max_confidence) {
        max_confidence = result.classification[i].value;
        predicted_index = i;
      }
    }
    ei_printf("\n");

    // 3. Tentukan snore_status berdasarkan label dengan skor tertinggi DAN ambang batas kepercayaan
    uint8_t snore_status = 0;
    if (predicted_index != -1 && 
        strcmp(result.classification[predicted_index].label, "snoring") == 0 && 
        result.classification[predicted_index].value > 0.8) { // <-- TAMBAHKAN KONDISI INI
      snore_status = 1;
    }

    // 4. Siapkan data untuk dikirim via BLE
    uint8_t data[5];
    data[0] = snore_status;
    uint32_t timestamp = millis() / 1000;
    memcpy(&data[1], &timestamp, 4);

    // 5. Kirim data via BLE
    myCharacteristic.notify(data, 5);
    
    // 6. Tampilkan pesan debug tambahan untuk verifikasi
    ei_printf("Detected sound: '%s' : confidence %.5f.\n", result.classification[predicted_index].label, max_confidence);
    ei_printf("Sending BLE data -> snore_status: %d, timestamp: %lu\n\n", snore_status, timestamp);

    print_results = 0;
  }
}
/* Edge Impulse audio functions */
static void pdm_data_ready_inference_callback(void) {
  int bytesAvailable = PDM.available();
  int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

  if (record_ready) {
    for (int i = 0; i < bytesRead >> 1; i++) {
      inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

      if (inference.buf_count >= inference.n_samples) {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
      }
    }
  }
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
  if (inference.buffers[0] == NULL) return false;

  inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
  if (inference.buffers[1] == NULL) {
    free(inference.buffers[0]);
    return false;
  }

  sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));
  if (sampleBuffer == NULL) {
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  PDM.onReceive(&pdm_data_ready_inference_callback);
  PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));
  
  if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
    ei_printf("Failed to start PDM!");
    return false;
  }

  PDM.setGain(127);
  record_ready = true;
  return true;
}

static bool microphone_inference_record(void) {
  if (inference.buf_ready == 1) {
    ei_printf("Error sample buffer overrun.\n");
    return false;
  }

  while (inference.buf_ready == 0) {
    delay(1);
  }

  inference.buf_ready = 0;
  return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}
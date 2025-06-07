#include <bluefruit.h>
#include <PDM.h>
#include <snore-detection-final_inferencing.h>

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

  // Send results periodically
  if (++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW) {
    // Get classification result (0: no snore, 1: snore)
    uint8_t snore_status = (result.classification[1].value > 0.7) ? 1 : 0;
    
    // Prepare BLE data
    uint8_t data[5];
    data[0] = snore_status;
    
    uint32_t timestamp = millis() / 1000;
    memcpy(&data[1], &timestamp, 4);

    // Send via BLE
    myCharacteristic.notify(data, 5);
    
    // Debug output
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
    Serial.print("DSP: ");
    Serial.print(result.timing.dsp);
    Serial.print(" ms, Classification: ");
    Serial.print(result.timing.classification);
    Serial.println(" ms");

    Serial.print("Snore: ");
    Serial.print(snore_status);
    Serial.print(", Confidence: ");
    Serial.print(result.classification[1].value, 5);
    Serial.println();

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
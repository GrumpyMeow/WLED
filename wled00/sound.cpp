#include "wled.h"
#include "src/dependencies/cqt/CQT.h"

#define SamplingFrequency 44100
#define BlockSize 1024  // In number of samples (1024 is maximum of esp-idf)
#define FrequencyBands 16 // Number of frequency bands

// For development
#define ESP32
#define WLED_SOUND_INMP441

#ifdef WLED_ENABLE_SOUND

#ifdef ESP8266
    #include <i2s.h>
#endif

#ifdef ESP32
    #include "driver/i2s.h"

    #ifdef WLED_SOUND_INMP441

    int32_t i2sBuffer[BlockSize];
    int inputsignal[BlockSize];
    int outputsignal[FrequencyBands];
    uint32_t maxAmp = 1 ;

    const i2s_port_t I2S_PORT = I2S_NUM_0; // Select I2S Port number 0
      const i2s_config_t i2s_config = {
          .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
          .sample_rate = SamplingFrequency,
          .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
          .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
          .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
          .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, 
          .dma_buf_count = 2,
          .dma_buf_len = BlockSize,
          .use_apll = false,
          .tx_desc_auto_clear = false,
          .fixed_mclk = 0,
      };

    const i2s_pin_config_t pin_config = {
          .bck_io_num = 14,   // INMP441 SCK  BCKL
          .ws_io_num = 15,    // INMP441 WS   LRCL
          .data_out_num = I2S_PIN_NO_CHANGE, 
          .data_in_num = 34   // INMP441 SD
          // INMP441 L/R -> GND
      };
    #endif
#endif

CQT *cqt;

void handleSound()
{
    // Read data from microphone
    size_t num_bytes_read;
    esp_err_t err = i2s_read(I2S_PORT, &i2sBuffer, sizeof(i2sBuffer), &num_bytes_read, portMAX_DELAY);
    if (err == ESP_OK && num_bytes_read != 0) {
        for (int i=0;i<BlockSize;i++) {
            int32_t sample = i2sBuffer[i];
            int32_t amp = sample;
            if (amp<0) amp = -amp;
            if (amp>maxAmp) maxAmp = amp;
            float normalizedSample = sample / (maxAmp/512.0f);
            inputsignal[i] = normalizedSample;
        }
        
        cqt->calculate(inputsignal, outputsignal);
        for (int i=0;i<FrequencyBands;i++) {
            Serial.printf("%d\t" , outputsignal[i]);
        }
        Serial.println();
    } 
}

void initSound() {
    #ifdef ESP32
    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }
    Serial.println("I2S driver installed.");
    #endif    

    cqt = new CQT();
    cqt->init( FrequencyBands , SamplingFrequency, BlockSize);
}

#else
    void handleSound() {}
    void initSound() {}
#endif

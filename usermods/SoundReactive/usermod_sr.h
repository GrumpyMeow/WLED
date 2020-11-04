#pragma once

#include "wled.h"
#include "driver/i2s.h"
#include "src/dependencies/cqt/CQT.h"

// I2S settings
#define I2S_NUM             I2S_NUM_0           // INMP441 L/R -> GND
#define I2S_BCK_IO_NUM      14                  // INMP441 SCK  BCKL
#define I2S_WS_IO_NUM       15                  // INMP441 WS   LRCL
#define I2S_DATA_OUT_NUM    I2S_PIN_NO_CHANGE
#define I2S_IN_NUM          32                  // INMP441 SD
#define I2S_BLOCK_SIZE      64

#define SampleBufferSize    NRSAMPLES * 2





class SoundReactiveUsermod : public Usermod {
  private:
    unsigned long lastTime = 0;

    // Constant Q Transform object
    CQT *cqt;

    uint16_t sampleIndex = 1;
    uint16_t fftIndex = 0;
    
    int sampleBuffer[SampleBufferSize];

    int32_t i2sBuffer[I2S_BLOCK_SIZE];
    const i2s_port_t I2S_PORT = I2S_NUM; 
    const i2s_config_t i2s_config = {
            .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate = FSAMPLE,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
            .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, 
            .dma_buf_count = 2,
            .dma_buf_len = I2S_BLOCK_SIZE
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO_NUM,   
        .ws_io_num = I2S_WS_IO_NUM,    
        .data_out_num = I2S_DATA_OUT_NUM, 
        .data_in_num = I2S_IN_NUM   
        
    };

    int32_t Sample_I2S();
    void initSound() ;
    void initCQT() ;
    void initTimers();
  public:
    // Public as to re-enter the object after callback  
    void sampleTimer_callback(void *arg);
    void fftTimer_callback(void *arg);

    void setup(); 
    void loop();
    uint16_t getId();    
};
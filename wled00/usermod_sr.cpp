#include "wled.h"
#include "usermod_sr.h"

void global_sampleTimer_callback(void *arg) {
  SoundReactiveUsermod* usermod = (SoundReactiveUsermod*)arg;
  usermod->sampleTimer_callback(arg);
}

void global_fftTimer_callback(void *arg) {
  SoundReactiveUsermod* usermod = (SoundReactiveUsermod*)arg;
  usermod->fftTimer_callback(arg);
}


#pragma region PRIVATE

  void SoundReactiveUsermod::initSound() {
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
  }

  void SoundReactiveUsermod::initCQT() {
    Serial.println("CQT initializing...");
    cqt = new CQT();
    cqt->init();
    
  }

  void SoundReactiveUsermod::initTimers() {
    // Setup Sampling timer
    esp_timer_create_args_t sampleTimer = {
      .callback = global_sampleTimer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "sample_timer"
    };
    esp_timer_handle_t sampleTimerHandle;
    ESP_ERROR_CHECK(esp_timer_create(&sampleTimer, &sampleTimerHandle));
    uint64_t samplePeriod = 1000000 / FSAMPLE ; 
    ESP_ERROR_CHECK(esp_timer_start_periodic(sampleTimerHandle, samplePeriod));

    // Setup FFT timer
    esp_timer_create_args_t fftTimer = {
      .callback = global_fftTimer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "fft_timer"
    };
    esp_timer_handle_t fftTimerHandle;
    ESP_ERROR_CHECK(esp_timer_create(&fftTimer, &fftTimerHandle));
    uint64_t fftPeriod = 1000000 / (FSAMPLE / (NRSAMPLES/4)) ; 
    ESP_ERROR_CHECK(esp_timer_start_periodic(fftTimerHandle, fftPeriod));

    Serial.println("Timers initialized.");
  }

int32_t maxAmp = 0;

  int SoundReactiveUsermod::Sample_I2S() {
    int32_t digitalSample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&digitalSample, portMAX_DELAY);
    if (bytes_read > 0) {    
        int32_t amp = abs(digitalSample);
        if (amp > maxAmp) maxAmp = amp;
        if (maxAmp>8192) {
          digitalSample = digitalSample / (maxAmp/ 8192);
        }
        maxAmp *=0.999f;
       

        return digitalSample ;
    };
  }
  
  void SoundReactiveUsermod::sampleTimer_callback(void *arg) {
    int digitalSample = Sample_I2S();
    sampleBuffer[sampleIndex] = digitalSample;

    sampleIndex++;
    if (sampleIndex>SampleBufferSize) {
      //Serial.print("X");
      sampleIndex = 0;
    }
  }

  void SoundReactiveUsermod::fftTimer_callback(void *arg) {
    int16_t length = 0;
    if (sampleIndex > fftIndex) {
      length = sampleIndex-fftIndex;
    } else if (fftIndex> sampleIndex) {
      length = SampleBufferSize - fftIndex + sampleIndex;
    }

    if (length > NRSAMPLES ) {
      for (int idx=0;idx<NRSAMPLES;idx++) {
        
        // signal values should be in range -512 ... +512
        int16_t latestSample = sampleBuffer[fftIndex];

        cqt->signal[(cqt->nrInterrupts) % NRSAMPLES] = latestSample;
        cqt->signal_lowfreq[(cqt->nrInterrupts / LOWFREQDIV) % NRSAMPLES] = latestSample;     

        cqt->nrInterrupts++;
        fftIndex++;
        if (fftIndex > SampleBufferSize) {
          //Serial.print("Y");
          fftIndex = 0;
        }
      }

      // for (int idx=0;idx<NRSAMPLES;idx++) {
      //   Serial.printf("%03d ", cqt->signal[idx]);
      // }
      // Serial.println("");

      // for (int idx=0;idx<NRSAMPLES;idx++) {
      //   Serial.printf("%04d ", cqt->signal_lowfreq[idx]);
      // }
      
      // Serial.println("");

      cqt->adjustInputs();
      cqt->cqt();
      for (int idx=0;idx<FREQS;idx++) {
        Serial.printf("%04d ", cqt->freqs[idx]);
      }
      Serial.println("");
    } 
  }
#pragma endregion

#pragma region PUBLIC
      
  void SoundReactiveUsermod::setup() {
    initSound();
    initCQT();
    initTimers();
  }

  void SoundReactiveUsermod::loop() {
    if (millis() - lastTime > 1000) {
      //Serial.println("I'm alive!");
      lastTime = millis();
    }
  }  

  uint16_t SoundReactiveUsermod::getId()
  {
    return USERMOD_ID_SOUNDREACTIVE;
  }

#pragma endregion


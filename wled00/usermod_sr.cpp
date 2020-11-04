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
    cqt->printFilters();    
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
    uint64_t fftPeriod = 1000000 / (FSAMPLE / NRSAMPLES); 
    ESP_ERROR_CHECK(esp_timer_start_periodic(fftTimerHandle, fftPeriod));

    Serial.println("Timers initialized.");
  }

int32_t maxAmp = 512;

  int SoundReactiveUsermod::Sample_I2S() {
    int32_t digitalSample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&digitalSample, portMAX_DELAY);
    if (bytes_read > 0) {    
        digitalSample = digitalSample >> 16;
        int32_t amp = abs(digitalSample);
        if (amp > maxAmp) maxAmp = amp;
        if (maxAmp>512) {
          digitalSample = (digitalSample*512) / maxAmp;
          maxAmp -= 1;
        }       

        return digitalSample ;
    };
  }
  
  void SoundReactiveUsermod::sampleTimer_callback(void *arg) {
    int digitalSample = Sample_I2S();
    cqt->sampleIndex++;  
    cqt->signal[(cqt->sampleIndex) % NRSAMPLES] = digitalSample;
    cqt->signal_lowfreq[(cqt->sampleIndex / LOWFREQDIV) % NRSAMPLES] = digitalSample;
  }

  void SoundReactiveUsermod::fftTimer_callback(void *arg) {
      const unsigned long startTime = millis();

      cqt->cqt();
      Serial.printf("%09d " , maxAmp);
      for (int idx=0;idx<NRBANDS;idx++) {
        int v = cqt->bandEnergy[idx];
        if (v>20) {
          Serial.printf("%03d ", v);
        } else { Serial.print(".   ");}
      }
      Serial.printf("%3dms\n", millis() - startTime);
  }
#pragma endregion

#pragma region PUBLIC
      
  void SoundReactiveUsermod::setup() {
    initSound();
    initCQT();
    initTimers();
  }

  void SoundReactiveUsermod::loop() {
    delay(1);
  }

  void SoundReactiveUsermod::addToJsonInfo(JsonObject& root)
  {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray amplitudeArr = user.createNestedArray("Max amplitude");
    amplitudeArr.add(maxAmp);

    JsonArray cqtBandsArr = user.createNestedArray("CQT bands");
    cqtBandsArr.add(NRBANDS);

    JsonArray i2sDataPinArr = user.createNestedArray("I2S data pin");
    i2sDataPinArr.add(-1);
  }


  void SoundReactiveUsermod::addToJsonState(JsonObject& root)
  {
    root["sampleRate"] = FSAMPLE;
    root["nrBands"] = NRBANDS;
    root["amplitude"] = cqt->amplitude;
    root["lowFreqBound"] = lowFreqBound;
    root["highFreqBound"] = highFreqBound;
  }


  /*
    * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
    * Values in the state object may be modified by connected clients
    */
  void SoundReactiveUsermod::readFromJsonState(JsonObject& root)
  {
    //FSAMPLE = root["sampleRate"] ;
    //cqt->amplitude = root["amplitude"] ;
    //root["lowerFreq"] = 20;
    //root["upperFreq"] = 11025;
  }  

  uint16_t SoundReactiveUsermod::getId()
  {
    return USERMOD_ID_SOUNDREACTIVE;
  }

#pragma endregion


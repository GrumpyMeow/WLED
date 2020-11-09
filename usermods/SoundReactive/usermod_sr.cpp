#include "wled.h"
#include "usermod_sr.h"

uint8_t NrBands = NRBANDS;
int BandEnergy[NRBANDS];
uint8_t reactiveIntensity;
bool reactivePeak;
uint8_t reactivePeakIntensity;

void global_sampleTimer_callback(void *arg) {
  SoundReactiveUsermod* usermod = (SoundReactiveUsermod*)arg;
  usermod->sampleTimer_callback(arg);
}

void global_transformTimer_callback(void *arg) {
  SoundReactiveUsermod* usermod = (SoundReactiveUsermod*)arg;
  usermod->transformTimer_callback(arg);
}


#pragma region PRIVATE

  void SoundReactiveUsermod::initSound() {
    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    #ifdef ESP32 
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
    #endif
    #ifdef ESP8266
      i2s_begin();
      i2s_set_rate(22050);
      
    #endif
    Serial.println("I2S driver installed.");
  }

  void SoundReactiveUsermod::initCQT() {
    Serial.println("CQT initializing...");
    cqt = new CQT();
    cqt->init();
    cqt->printFilters();    
  }

  void SoundReactiveUsermod::initAGC() {
    Serial.println("AGC initializing...");
    agc = new autogaincontrol();
    agc->init();
  }

  void SoundReactiveUsermod::initTimers() {
    #ifdef ESP32
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

    // Setup (CQT/FFT) Transform timer
    esp_timer_create_args_t transformTimer = {
      .callback = global_transformTimer_callback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "transform_timer"
    };
    esp_timer_handle_t transformTimerHandle;
    ESP_ERROR_CHECK(esp_timer_create(&transformTimer, &transformTimerHandle));
    uint64_t transformPeriod = 1000000 / (FSAMPLE / NRSAMPLES); 
    ESP_ERROR_CHECK(esp_timer_start_periodic(transformTimerHandle, transformPeriod));

    Serial.println("Timers initialized.");
    #endif
  }

int32_t maxAmp = 512;

  #ifdef ESP32
  int SoundReactiveUsermod::Sample_I2S() {
    int32_t digitalSample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&digitalSample, portMAX_DELAY);
    if (bytes_read > 0) {
        // INMP441 scale down
        digitalSample = digitalSample >> 16; // Scale dow

        return digitalSample ;
    };
  }
  #endif
  
  void SoundReactiveUsermod::sampleTimer_callback(void *arg) {
    int digitalSample = Sample_I2S();

    //int agcSample = agc->process(digitalSample);
    digitalSample =  digitalSample * agcMultiplier;

    cqt->sampleIndex++;  
    cqt->signal[(cqt->sampleIndex) % NRSAMPLES] = digitalSample;
    cqt->signal_lowfreq[(cqt->sampleIndex / LOWFREQDIV) % NRSAMPLES] = digitalSample;
    reactiveIntensity = abs(digitalSample)>255 ? 255: abs(digitalSample) ;

    // Process samples for amplitude information
    uint rawAmp = abs(digitalSample);
    totalRawRMS += (rawAmp*rawAmp) ;
    if (rawAmp>totalRawPeak) {
      totalRawPeak = rawAmp;
    }

    // update RMS and Peak
    if (cqt->sampleIndex % NRSAMPLES==0) {      
      currentRawPeak = totalRawPeak;
      int newRawRMS = sqrt(totalRawRMS/ NRSAMPLES);
      currentRawRMS = newRawRMS;      
      totalRawRMS = 0;
      totalRawPeak = 0;

      //agcMultiplier = agcMultiplier+ ((100 - newRawRMS) / 10);      
    }
  }

  void SoundReactiveUsermod::transformTimer_callback(void *arg) {
      const unsigned long startTime = millis();

      cqt->cqt();      
      int totalEnergy = 0;
      for (uint8_t band=0;band<NRBANDS;band++) {
        totalEnergy += cqt->bandEnergy[band];
      }      
      int avgEnergy = totalEnergy / NRBANDS;
      Serial.printf("%3d %4d %3d %f  | ", avgEnergy, currentRawPeak, currentRawRMS, agcMultiplier /*agc->AgcMultiplier() */);
      for (uint8_t band=0;band<NRBANDS;band++) {
        
        int bandEnergy = cqt->bandEnergy[band];
        bandEnergy -= avgEnergy;
        if (bandEnergy<0) bandEnergy = 0;
        BandEnergy[band] = bandEnergy;

        Serial.printf("%3d",bandEnergy);
      }
      Serial.println();
  }
#pragma endregion

#pragma region PUBLIC
      
  void SoundReactiveUsermod::setup() {
    
  }

  void SoundReactiveUsermod::loop() {
    if (!initialized) {
      initialized = true;
      initAGC();
      initSound();
      initCQT();
      initTimers();
    }
    delay(1);
  }

  void SoundReactiveUsermod::addToJsonInfo(JsonObject& root)
  {
    // Add information to the Info-page
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray sreArr = user.createNestedArray("Sound reactive");
    sreArr.add("Enabled");

    // Add information to the Info-json-object
    JsonObject rve = root.createNestedObject("rve");
    double crest = currentRawPeak / (double)currentRawRMS;
    rve[F("Peak/RMS/Crest")] = String(currentRawPeak) + "/" + String(currentRawRMS) + "/" + String(crest,3) ;
    rve[F("Frequency bands")] = NRBANDS;
    rve[F("I2S data pin")] = -1;
    rve[F("Low Frequency Bound")] = LOWFREQBOUND;
    rve[F("High Frequency Bound")] = HIGHFREQBOUND;

    JsonObject freqEnergy = root.createNestedObject("FreqEnergy");        
    for (uint band=0;band<NRBANDS;band++) {
      freqEnergy[String(cqt->Freq[band])]= cqt->bandEnergy[band];
    }
  }


  void SoundReactiveUsermod::addToJsonState(JsonObject& root)
  {
    JsonObject rve = root.createNestedObject("rve");    
    rve["amplitude"] = cqt->Amplitude();
    rve["lowFreq"] = cqt->LowFreq();
    rve["highFreq"] = cqt->HighFreq();
    rve["autoGain"] = -1;
  }


  void SoundReactiveUsermod::readFromJsonState(JsonObject& root)
  {
    JsonObject rve = root["rve"];
    if (rve) {
      uint amplitude = rve["amplitude"] ;
      uint lowFreq = uint( rve["lowFreqBound"]);
      uint highFreq = uint(rve["highFreqBound"]);
      cqt->adjustInputs(amplitude, lowFreq, highFreq);
    }
  }  

  uint16_t SoundReactiveUsermod::getId()
  {
    return USERMOD_ID_SOUNDREACTIVE;
  }

#pragma endregion


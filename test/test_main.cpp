#include <unity.h>
#include <Arduino.h>
#include "src/dependencies/cqt/cqt.h"
#include "src/dependencies/cqt/cqt.cpp"

void test_cqt_totalsilence(void) {
    CQT* cqt = new CQT();
    cqt->init();    
    cqt->printFilters();    
    for (int i=0;i<NRSAMPLES;i++) {
        int sample = 0;
        cqt->sampleIndex++;  
        cqt->signal[(cqt->sampleIndex) % NRSAMPLES] = sample;
        cqt->signal_lowfreq[(cqt->sampleIndex / LOWFREQDIV) % NRSAMPLES] = sample;
    }
    cqt->cqt();
    for (int i=0;i<NRBANDS;i++) {
        Serial.printf("%4d ",cqt->bandEnergy[i]);
    }
    Serial.println("");    
}

void test_cqt_Sin100Hz(void) {
    CQT* cqt = new CQT();
    cqt->init();        
    
    double angle = 0;
    const double frequencyHz = 4000.0f;
    const double amplitude = 128.0f;
    double freqinc = (((PI*2.0f) / FSAMPLE) * frequencyHz);
    for (int i=0;i<NRSAMPLES;i++) {
        int sample = amplitude * sin(angle);
        cqt->sampleIndex++;  
        cqt->signal[(cqt->sampleIndex) % NRSAMPLES] = sample;
        cqt->signal_lowfreq[(cqt->sampleIndex / LOWFREQDIV) % NRSAMPLES] = sample;
        angle += freqinc;
    }
    uint begin = millis();
    cqt->cqt();
    Serial.printf("CQT Time=%d ms\n", millis()-begin);

    for (int i=0;i<NRBANDS;i++) {
        Serial.printf("%4d ",cqt->bandEnergy[i]);
    }
    Serial.println("");    
}

void test_cqt_Sin200Hz(void) {
    CQT* cqt = new CQT();
    cqt->init();        
    
    double angle = 0;
    const double frequencyHz = 2000.0f;
    const double amplitude = 128.0f;
    double freqinc = (((PI*2.0f) / FSAMPLE) * frequencyHz);
    for (int i=0;i<NRSAMPLES;i++) {
        int sample = amplitude * sin(angle);
        cqt->sampleIndex++;  
        cqt->signal[(cqt->sampleIndex) % NRSAMPLES] = sample;
        cqt->signal_lowfreq[(cqt->sampleIndex / LOWFREQDIV) % NRSAMPLES] = sample;
        angle += freqinc;
        
    }
    uint begin = millis();
    cqt->cqt();
    Serial.printf("CQT Time=%d ms\n", millis()-begin);

    for (int i=0;i<NRBANDS;i++) {
        Serial.printf("%4d ",cqt->bandEnergy[i]);
    }
    Serial.println("");    
}

void setup() {
    Serial.begin(115200);
    UNITY_BEGIN();
    RUN_TEST(test_cqt_totalsilence);
    RUN_TEST(test_cqt_Sin100Hz);
    RUN_TEST(test_cqt_Sin200Hz);
    
    UNITY_END(); 
}

void loop() {
}
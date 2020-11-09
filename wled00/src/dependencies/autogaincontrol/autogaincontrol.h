#ifndef autogaincontrol_h
#define autogaincontrol_h

class autogaincontrol {
public:
    float AgcMultiplier() const { return multAgc; }

    void init();    
    int process(int sample);
    void print();
private:
    int sample;
    int prevAmplitude;
    float multAgc;                                      // sample * multAgc = sampleAgc. Our multiplier
    uint8_t targetAgc = 60;                             // Target Auto Gain Control
    byte sampleGain = 1;
    float sampleAvg = 0;                                // Smoothed Average
    byte soundSquelch = 10;
};

#endif

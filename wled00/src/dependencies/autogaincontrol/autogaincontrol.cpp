
/* ----- LIBRARIES ----- */
#include <Arduino.h>

#include "autogaincontrol.h"


// Constructor of autogaincontrol
void autogaincontrol::init() {
}

int autogaincontrol::process(int micIn) {
  // Using a ternary operator, the result is either 0 (when amplitude below soundSquelch (noise)) or it's a bit smoothed out with the previous amplitude.
  int amplitude = abs(micIn);
  int result = (amplitude <= soundSquelch) ? 0 : ((prevAmplitude * 3) + amplitude) / 4;

  int sampleAdj = ((result * sampleGain) / 40) + (result / 16); // Adjust the gain.
  prevAmplitude = min(sampleAdj, 255);;                         // We'll now make our rebase our sample to be adjusted.

  sampleAvg = ((sampleAvg * 15) + sampleAdj) / 16;       			// Smooth it out over the last samples.
  
  multAgc = (sampleAvg < 1) ? targetAgc : targetAgc / sampleAvg;// Make the multiplier so that sampleAvg * multiplier = setpoint

  return (micIn * multAgc);
}

void autogaincontrol::print() {
	//Serial.printf("%d %f %f %f\n", sample, sampleAvg, multAgc);
}

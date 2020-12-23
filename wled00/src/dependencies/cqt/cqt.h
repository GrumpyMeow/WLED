#ifndef CQT_h
#define CQT_h


// Define constants
#define LOWFREQDIV 8              // Lower-sampling-frequency divider
#define NRBANDS 16                // Number of frequency bands to calculate
#define FSAMPLE 44100             // SampleRate of input
#define LOWFREQBOUND 20           // Lower frequency of interest
#define HIGHFREQBOUND 500        // Upper frequency of interest
#define NRSAMPLES 1024            // Size of samplebbufer
#define MAXTOTALSAMPLES 2048      // Total number of samples sizeof(signal + signal_lowfreq)

class CQT {
public:
  uint LowFreq() const { return F0; }
  uint HighFreq() const { return Fmax; }
  unsigned int Freq[NRBANDS + 1];	// TODO: This should be private.
  
  int signal[NRSAMPLES];		      // [input] Array of samples
  int signal_lowfreq[NRSAMPLES];	// [input] Array of samples, with a lower sampling frequency (see LOWFREQDIV)
  float bandEnergy[NRBANDS];	      // [output] Calculated energy for each band/filter
  unsigned int sampleIndex;       // Counter of number of interrupts. Used for looping over the input-buffer
  void init();                    
  void cqt();
  void adjustInputs(uint newLowFreq, uint newHighFreq);
  void printFilters();
private:
  uint F0;                        // Lower frequency of range of interest
  uint Fmax;	                    // Upper frequency of range of interest
  uint32_t lowfreqEndIndex;       // Index of the band at which the lowfreq buffer is used
  float TAU =  (2*PI);

  unsigned int Div[NRBANDS];	    // Sample frequency divider for each filter
  unsigned int NFreq[NRBANDS];	  // Number of samples needed for each filter
  
  void preprocess_filters();
  float hamming(int m, int k);     // Hamming window
};

#endif
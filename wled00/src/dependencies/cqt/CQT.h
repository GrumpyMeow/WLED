#ifndef CQT_h
#define CQT_h

// Define type aliases
#define fixedpoint int            // Fixedpoint value (multiplied by PRECISION)

// Define constants
#define QPOSCIRCLE 13             // Fixed point bits (2*PI)
#define int2PI (1<<QPOSCIRCLE)    // So in our book, a circle is a full 8192 units long. The sinus functions is based on this property!
#define PRECISION 10              // NOTE: Higher than 10 might give overflow for 32-bit numbers when multiplying... 10 = (*1024)
#define LOWFREQDIV 8              // Lower-sampling-frequency divider
#define ALPHA ((7<<PRECISION)/QPOSCIRCLE)	// 0.53836*(1<<PRECISION)       == 7168 / 13 = 551
#define BETA ((6<<PRECISION)/QPOSCIRCLE)	// 1-0.53836*(1<<PRECISION)     == 6144 / 13 = 472
#define SCALE (1<<PRECISION)      // == 1024
#define NRBANDS 16                // Number of frequency bands to calculate
#define FSAMPLE 22050             // SampleRate of input
#define LOWFREQBOUND 20           // Lower frequency of interest
#define HIGHFREQBOUND 4000        // Upper frequency of interest
#define DEFAULT_AMPLITUDE 1024       
#define NRSAMPLES 1024            // Size of samplebbufer
#define MAXTOTALSAMPLES 2048      // Total number of samples sizeof(signal + signal_lowfreq)

class CQT {
public:
  uint LowFreq() const { return F0; }
  uint HighFreq() const { return Fmax; }
  uint Amplitude() const { return amplitude; }
  unsigned int Freq[NRBANDS + 1];	// TODO: This should be private.
  
  int signal[NRSAMPLES];		      // [input] Array of samples
  int signal_lowfreq[NRSAMPLES];	// [input] Array of samples, with a lower sampling frequency (see LOWFREQDIV)
  int bandEnergy[NRBANDS];	      // [output] Calculated energy for each band/filter
  unsigned int sampleIndex;       // Counter of number of interrupts. Used for looping over the input-buffer
  void init();                    
  void cqt();
  void adjustInputs(uint newAmplitude, uint newLowFreq, uint newHighFreq);
  void printFilters();
private:
  uint F0;                        // Lower frequency of range of interest
  uint Fmax;	                    // Upper frequency of range of interest
  uint amplitude;	                // Amplification of calculated bandEnergy [0-1024]
  uint32_t lowfreqEndIndex;       // Index of the band at which the lowfreq buffer is used
  unsigned fixedpoint twoPiQ;	    // Calculated 2*pi*Q value for the CQT (Constant Q Transform)

  unsigned int Div[NRBANDS];	    // Sample frequency divider for each filter
  unsigned int NFreq[NRBANDS];	  // Number of samples needed for each filter
  
  void preprocess_filters();

  fixedpoint approxSin(int x);          // Approximate SIN value
  fixedpoint approxCos(fixedpoint in);  // Approximate COS value
  fixedpoint hamming(int m, int k);     // Hamming window
};

#endif

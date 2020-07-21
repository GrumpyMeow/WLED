#include <inttypes.h>

// Constant Q Transform
// Frequency bins (11025 Hz,16 bands)= 20--28--40--57--81--115--164--233--471--670--952--1352--1922--2730--3879--5512


#ifndef CQT_h
#define CQT_h

// ---- Methods ----

#define int2PI (1<<13)		// So in our book, a circle is a full 8192 units long. The sinus functions is based on this property!
#define fixedpoint int
#define PRECISION 10    // NOTE: Higher than 10 might give overflow for 32-bit numbers when multiplying...
#define LOWFREQDIV 8
#define ALPHA ((7<<PRECISION)/13)	// 0.53836*(1<<PRECISION)
#define BETA ((6<<PRECISION)/13)	// 1-0.53836*(1<<PRECISION)
#define SCALE (1<<PRECISION)


class CQT {
public:
  void init(uint FrequencyBands, uint SampleRate, uint BlockSize);
  int * calculate(int32_t *signal);
private:
  uint32_t lowfreq_endIndex = 0;
  uint32_t amplitude;			// amplification of signal
  unsigned fixedpoint twoPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)
  unsigned int FrequencyBands; // Number of frequency bands
  unsigned int SampleRate; // Sample rate in Herz
  unsigned int BlockSize; // Block size of sampled data
  unsigned int * Div;	// sample frequency divider for each filter
  unsigned int * BandFrequency; // Frequency edges of bands
  unsigned int * NyqistFrequency; // Nyqist frequencies per band
  int * signal_lowfreq;	// current sample signal, with a lower samplin frequency (see LOWFREQDIV)
  int * Output;
  unsigned int lowFreqBound; // Lower frequency bound
  unsigned int highFreqBound; // Higher frequency bound
  unsigned int minFreq; 
  unsigned int maxFreq;
  void calculateParameters(uint minFreq, uint maxFreq);
  int approxSin(int x);
  fixedpoint approxCos(fixedpoint in);
  fixedpoint hamming(int m, int k);
};

#endif

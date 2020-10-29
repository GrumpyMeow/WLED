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
#define FREQS 16    // Number of frequency bands


class CQT {
public:
  int * signal;     // current sample signal
  int freqs[FREQS];		// frequencies for each band/filter
  void init(uint SampleRate, uint BlockSize);
  void calculate();
private:
  unsigned int nrInterrupts;
  uint NRSAMPLES;
  uint MAXTOTALSAMPLES;
  uint32_t lowfreq_endIndex = 0;
  unsigned int Freq[FREQS + 1];	// lower and upper frequency for each filter/band
  unsigned int Div[FREQS];	// sample frequency divider for each filter
  unsigned int NFreq[FREQS];	// number of samples needed for each filter

  uint32_t amplitude;			// amplification of signal
  unsigned fixedpoint twoPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)
  unsigned int SampleRate; // Sample rate in Herz  
  int * signal_lowfreq;	// current sample signal, with a lower samplin frequency (see LOWFREQDIV)
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

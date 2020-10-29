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
#define FSAMPLE 14200
#define lowFreqBound 20
#define highFreqBound 7000
#define NRSAMPLES (1024)
#define MAXTOTALSAMPLES 2048


class CQT {
public:
  int signal[NRSAMPLES];		// current sample signal
  int signal_lowfreq[NRSAMPLES];	// current sample signal, with a lower samplin frequency (see LOWFREQDIV)
  int freqs[FREQS];		// frequencies for each band/filter
  uint32_t nrInterrupts;
  void init();
  void cqt();
  void adjustInputs();
private:
  unsigned int oldMinF, oldMaxF, F0, F16;	// minimum and maximum input frequencies
  
  uint32_t lowfreq_endIndex = 0;

  unsigned int Freq[FREQS + 1];	// lower and upper frequency for each filter/band
  unsigned int Div[FREQS];	// sample frequency divider for each filter
  unsigned int NFreq[FREQS];	// number of samples needed for each filter
  unsigned fixedpoint twoPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)

  uint32_t amplitude = 512;			// amplification of signal
  unsigned int minFreq; 
  unsigned int maxFreq;
  void preprocess_filters();
  int approxSin(int x);
  fixedpoint approxCos(fixedpoint in);
  fixedpoint hamming(int m, int k);
};

#endif

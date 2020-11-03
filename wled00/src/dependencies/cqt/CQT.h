#include <inttypes.h>

#ifndef CQT_h
#define CQT_h

// ---- Methods ----

#define fixedpoint int
#define u32 uint32_t

#define int2PI (1<<13)		// So in our book, a circle is a full 8192 units long. The sinus functions is based on this property!
#define PRECISION 10    // NOTE: Higher than 10 might give overflow for 32-bit numbers when multiplying... 10 = (*1024)
#define LOWFREQDIV 8
#define ALPHA ((7<<PRECISION)/13)	// 0.53836*(1<<PRECISION)       == 7168 / 13 = 551
#define BETA ((6<<PRECISION)/13)	// 1-0.53836*(1<<PRECISION)     == 6144 / 13 = 472
#define SCALE (1<<PRECISION)      // == 1024
#define FREQS 16    // Number of frequency bands
#define FSAMPLE 22050
#define lowFreqBound 132
#define highFreqBound 3960
#define NRSAMPLES 1024
#define MAXTOTALSAMPLES 2048


class CQT {
public:
  int signal[NRSAMPLES];		// current sample signal
  int signal_lowfreq[NRSAMPLES];	// current sample signal, with a lower samplin frequency (see LOWFREQDIV)
  int freqs[FREQS];		// frequencies for each band/filter
  unsigned int nrInterrupts;
  void init();
  void cqt();
  void adjustInputs();
private:
  unsigned int F0, F16;	// minimum and maximum input frequencies
  
  u32 lowfreq_endIndex = 0;

  unsigned int Freq[FREQS + 1];	// lower and upper frequency for each filter/band
  unsigned int Div[FREQS];	// sample frequency divider for each filter
  unsigned int NFreq[FREQS];	// number of samples needed for each filter
  unsigned fixedpoint twoPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)

  u32 amplitude = 256;			// amplification of signal
  void preprocess_filters();
  int32_t approxSin(int32_t x);
  int32_t approxCos(int32_t in);
  int32_t hamming(int32_t m, int32_t k);
};

#endif

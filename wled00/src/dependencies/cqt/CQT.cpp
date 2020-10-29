
/* ----- LIBRARIES ----- */
#include <Arduino.h>

#include "CQT.h"


// Constructor of CQT
void CQT::init() {
	this->nrInterrupts = 0;

	for (uint k = 0; k < NRSAMPLES; ++k) {
	    signal[k] = 0;		
		signal_lowfreq[k] = 0;
    }

    // TODO: initialize everything decently to avoid division by 0 errors and other problems
    for (uint k = 0; k < FREQS; ++k) {
	    Div[k] = 1;		
    }

	adjustInputs();
}

void CQT::adjustInputs() {

    unsigned int minF = 0; // readpin
    unsigned int maxF = 44100; // readpin

	float base = 1.0091;
	minF = powf(base, minF);
	maxF = powf(base, maxF);
	if (minF < lowFreqBound) {
		minF = lowFreqBound;
	}
	if (maxF > highFreqBound) {
		maxF = highFreqBound;
	}
	if (2 * minF > maxF) {	// at least one octave will always be displayed.
		if (2 * minF > highFreqBound) {
			maxF = highFreqBound;
			minF = highFreqBound / 2;
		} else {
			maxF = 2 * minF;
		}
	}
	if (maxF - minF < 60) {	// low frequencies must differ at least 60, so that the frequency width of each bucket is at least 2Hz. This prevents the number of samples needed to be larger than MAXSAMPLESIZE
		maxF = minF + 60;
	}

	F0 = minF;
	F16 = maxF;
	//Serial.printf("Freq range: %d Hz - %d Hz\n",F0,F16);
	preprocess_filters();
}

void CQT::preprocess_filters() {
    float nn = powf(2, log(F16 / (double) F0) / log(2) / 16.0);
    twoPiQ = int2PI * (nn + 1) / (nn - 1) / 2;
	Serial.printf("twoPiQ: %d\t int2PI=%d\t nn=%d\n", twoPiQ, int2PI, nn);

    int i;
    //printf("calculating the border frequencies...\n");
    for (i = 0; i < FREQS + 1; ++i) {
		Freq[i] = (F0 * powf(nn, i) + F16 / powf(nn, FREQS - i)) / 2;
    }

    //CDC.printf("calculating the index until which the signal_lowfreq samples should be used\n");
    lowfreq_endIndex = 0;
    while (FSAMPLE / (Freq[lowfreq_endIndex + 1] - Freq[lowfreq_endIndex]) >= NRSAMPLES && lowfreq_endIndex < FREQS) {
		++lowfreq_endIndex;
    }
	Serial.printf("LowFreqEndIndex: %d\n",lowfreq_endIndex);

    int samplesLeft = MAXTOTALSAMPLES;

    for (i = 16; i > lowfreq_endIndex; --i) {
		Div[i - 1] = 1 + FSAMPLE / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = FSAMPLE / (Freq[i] - Freq[i - 1]) / Div[i - 1];
		samplesLeft -= NFreq[i - 1];
    }
    for (; i > 0; --i) {
		Div[i - 1] = 1 + FSAMPLE / LOWFREQDIV / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = FSAMPLE / LOWFREQDIV / (Freq[i] - Freq[i - 1]) / Div[i - 1];
		samplesLeft -= NFreq[i - 1];
    }
	Serial.printf("samplesLeft: %d\n",samplesLeft);

	for (i=0;i < FREQS;++i) {
		Serial.printf("band=%d\tfreq=%d\tdiv=%d\tnfreq=%d\n", i, Freq[i], Div[i], NFreq[i]);
	}
}
// F0 = 20 F16=7000
// twoPiQ: 22624    int2PI=8192     nn=1610612736
// LowFreqEndIndex: 2
// samplesLeft: 315
// band=0  freq=19 		   div=1   nfreq=197
// band=1  freq=28 		   div=1   nfreq=136
// band=2  freq=41 		   div=3   nfreq=262
// band=3  freq=59 		   div=2   nfreq=262
// band=4  freq=86 		   div=2   nfreq=186
// band=5  freq=124        div=2   nfreq=129
// band=6  freq=179        div=1   nfreq=177
// band=7  freq=259        div=1   nfreq=123
// band=8  freq=374        div=1   nfreq=86
// band=9  freq=539        div=1   nfreq=59
// band=10 freq=778        div=1   nfreq=41
// band=11 freq=1122       div=1   nfreq=28
// band=12 freq=1618       div=1   nfreq=19
// band=13 freq=2333       div=1   nfreq=13
// band=14 freq=3365       div=1   nfreq=9
// band=15 freq=4853       div=1   nfreq=6



#define qN 11
#define qA PRECISION
#define qP 15
#define qR (2*qN-qP)
#define qS (qN+qP+1-qA)

int CQT::approxSin(int x) {
    // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
    // n : Q-pos for quarter circle             11, so full circle is 2^13 long
    // A : Q-pos for output                     10
    // p : Q-pos for parentheses intermediate   15
    // r = 2n-p                                  7
    // s = A-1-p-n                              17

    x = x << (30 - qN);		// resize to pi range
    // shift to full s32 range (Q13->Q30)

    if ((x ^ (x << 1)) < 0)	// test for quadrant 1 or 2
	x = (1 << 31) - x;

    x = x >> (30 - qN);
    return (x * ((3 << qP) - (x * x >> qR)) >> qS);
}

fixedpoint CQT::approxCos(fixedpoint in) {
    return approxSin((int2PI >> 2) - in);
}


fixedpoint CQT::hamming(int m, int k) {
    return ALPHA - (BETA * approxCos(int2PI * m / NFreq[k]) >> PRECISION);
}

void CQT::cqt() {
    unsigned int k, i, indx;
    int windowed, angle;
    float real_f, imag_f;
    fixedpoint real, imag;

	uint32_t nrInt = nrInterrupts;
    for (k = 0; k < lowfreq_endIndex; ++k) {
	indx = nrInt % NRSAMPLES - 1 + 8 * NRSAMPLES;
	real = ALPHA - (BETA * signal_lowfreq[indx % NRSAMPLES] >> PRECISION);
	imag = 0;
	for (i = 1; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * signal_lowfreq[(indx - i * Div[k]) % NRSAMPLES];
	    angle = twoPiQ * i / NFreq[k];
	    real += windowed * approxCos(angle) >> PRECISION;
	    imag += windowed * approxSin(angle) >> PRECISION;
	}

	real_f = real / (float) SCALE;
	imag_f = imag / (float) SCALE;
	freqs[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
    for (; k < FREQS; ++k) {
	indx = nrInt % NRSAMPLES - 1 + 8 * NRSAMPLES;
	real = ALPHA - (BETA * signal[indx % NRSAMPLES] >> PRECISION);
	imag = 0;
	for (i = 1; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * signal[(indx - i * Div[k]) % NRSAMPLES];
	    angle = twoPiQ * i / NFreq[k];
	    real += windowed * approxCos(angle) >> PRECISION;
	    imag += windowed * approxSin(angle) >> PRECISION;
	}
	real_f = real / (float) SCALE;
	imag_f = imag / (float) SCALE;
	freqs[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
    
}




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
	preprocess_filters();
}

void CQT::preprocess_filters() {
    //CDC.printf("calculating twoPiQ...\n");
    float nn = powf(2, log(F16 / (double) F0) / log(2) / 16.0);
    twoPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

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

    int samplesLeft = MAXTOTALSAMPLES;

    //CDC.printf("calculating sample frequency dividers...\n");
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
}


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

// Returns in range of 0..1024 
fixedpoint CQT::approxCos(fixedpoint in) {
    return approxSin((int2PI >> 2) - in);
}

// Returns values in range of 0..1024
fixedpoint CQT::hamming(int m, int k) {
	return ALPHA - (BETA * approxCos(int2PI * m / NFreq[k]) >> PRECISION);
}

void CQT::cqt() {
    unsigned int k, i, indx;
    int windowed, angle;
    float real_f, imag_f;
    fixedpoint real, imag;
    for (k = 0; k < lowfreq_endIndex; ++k) {
	indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
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
	indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
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



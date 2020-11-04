
/* ----- LIBRARIES ----- */
#include <Arduino.h>

#include "CQT.h"


// Constructor of CQT
void CQT::init() {
	this->sampleIndex = 0;

	for (uint k = 0; k < NRSAMPLES; ++k) {
	    signal[k] = 0;		
		signal_lowfreq[k] = 0;
    }

    // TODO: initialize everything decently to avoid division by 0 errors and other problems
    for (uint k = 0; k < NRBANDS; ++k) {
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
	Fmax = maxF;
	preprocess_filters();
}

void CQT::printFilters() {
	Serial.println("Constant Q Transform bands:");
	int band;
	for (band = 0; band<NRBANDS-1;band++) {
		if (band==lowfreqEndIndex) Serial.println("--- LOWFREQ sample-buffer used for above bands --- Normal sample-buffer used for below bands  ---");
		Serial.printf("Band:%3d\t\t%5dHz - %5dHz\tDiv=%d\tNFreq=%d\n", band , Freq[band], Freq[band+1], Div[band], NFreq[band] );
	}
	Serial.printf("Band:%3d\t\t%5dHz - %5dHz\tDiv=%d\tNFreq=%d\n", band , Freq[band], Fmax , Div[band], NFreq[band] );
}

void CQT::preprocess_filters() {
    float nn = powf(2, log(Fmax / (double) F0) / log(2) / (double)NRBANDS);
    twoPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

    int band;
    // calculating the border frequencies
    for (band = 0; band < NRBANDS + 1; ++band) {
		Freq[band] = (F0 * powf(nn, band) + Fmax / powf(nn, NRBANDS - band)) / 2;
    }

    // calculating the index until which the signal_lowfreq samples should be used
    lowfreqEndIndex = 0;
    while (FSAMPLE / (Freq[lowfreqEndIndex + 1] - Freq[lowfreqEndIndex]) >= NRSAMPLES && lowfreqEndIndex < NRBANDS) {
		++lowfreqEndIndex;
    }

    int samplesLeft = MAXTOTALSAMPLES;

    // calculating sample frequency dividers
    for (band = NRBANDS; band > lowfreqEndIndex; --band) {
		int bandFreqWidth = Freq[band] - Freq[band - 1];
		Div[band - 1] = 1 + FSAMPLE / (bandFreqWidth * samplesLeft / band);
		NFreq[band - 1] = FSAMPLE / bandFreqWidth / Div[band - 1];
		samplesLeft -= NFreq[band - 1];
    }

    for (; band > 0; --band) {
		int bandFreqWidth = Freq[band] - Freq[band - 1];
		Div[band - 1] = 1 + FSAMPLE / LOWFREQDIV / (bandFreqWidth * samplesLeft / band);
		NFreq[band - 1] = FSAMPLE / LOWFREQDIV / bandFreqWidth / Div[band - 1];
		samplesLeft -= NFreq[band - 1];
    }
}

// Q-pos for quarter circle (=FullCircle/4) = 11
#define qN (QPOSCIRCLE - 2)

// A : Q-pos for output
#define qA PRECISION

// p : Q-pos for parentheses intermediate (15)
#define qP 15

// r = 2n-p (7)
#define qR (2*qN-qP)

// s = A-1-p-n (17)
#define qS (qN+qP+1-qA)

// Returns in range of 0..1024 
fixedpoint CQT::approxSin(int x) {
    // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s

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
    for (k = 0; k < lowfreqEndIndex; ++k) {
	indx = sampleIndex % NRSAMPLES - 1 + 8 * NRSAMPLES;
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
	bandEnergy[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
    for (; k < NRBANDS; ++k) {
	indx = sampleIndex % NRSAMPLES - 1 + 8 * NRSAMPLES;
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
	bandEnergy[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
}



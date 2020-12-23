
/* ----- LIBRARIES ----- */
#include <Arduino.h>

#include "cqt.h"


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

	adjustInputs(LOWFREQBOUND, HIGHFREQBOUND);
}

void CQT::adjustInputs(uint newLowFreq, uint newHighFreq) {

    uint minF = newLowFreq; 
    uint maxF = newHighFreq; 

	float base = 1.0091;
	minF = powf(base, minF);
	maxF = powf(base, maxF);
	if (minF < LOWFREQBOUND) {
		minF = LOWFREQBOUND;
	}
	if (maxF > HIGHFREQBOUND) {
		maxF = HIGHFREQBOUND;
	}
	if (2 * minF > maxF) {	// at least one octave will always be displayed.
		if (2 * minF > HIGHFREQBOUND) {
			maxF = HIGHFREQBOUND;
			minF = HIGHFREQBOUND / 2;
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

float CQT::hamming(int m, int k) {
	auto const a = 25.0 / 46.0;
	return a - (1-a) * cosf(TAU * m) / NFreq[k];
}

void CQT::cqt() {
    unsigned int k, i, indx;
    float windowed;
	float angle;
    float real_f, imag_f;
    float real, imag;
    for (k = 0; k < lowfreqEndIndex; ++k) {
	indx = sampleIndex % NRSAMPLES - 1 + 8 * NRSAMPLES; // this seems wrong
	real = 0;
	imag = 0;
	for (i = 0; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * (float)signal_lowfreq[(indx - i * Div[k]) % NRSAMPLES];
	    angle = TAU * (float)i / (float)NFreq[k];
	    real += windowed * cosf(angle);
	    imag += windowed * sinf(angle);
	}

	real_f = real ;
	imag_f = imag ;
	bandEnergy[k] = powf(real_f * real_f + imag_f * imag_f, 0.5) / (float)NFreq[k];
    }

    for (; k < NRBANDS; ++k) {
	indx = sampleIndex % NRSAMPLES - 1 + 8 * NRSAMPLES; // this seems wrong
	real=0;
	imag = 0;
	for (i = 0; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * (float)signal[(indx - i * Div[k]) % NRSAMPLES];
		angle = TAU * (float)i / (float)NFreq[k];
	    real += windowed * cosf(angle);
	    imag += windowed * sinf(angle);
	}
	real_f = real ;
	imag_f = imag ;
	bandEnergy[k] = powf(real_f * real_f + imag_f * imag_f, 0.5) / (float)NFreq[k] ;
    }
}


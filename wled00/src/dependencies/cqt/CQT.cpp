
/* ----- LIBRARIES ----- */
#include <Arduino.h>

#include "CQT.h"



void CQT::init(uint FrequencyBands, uint SampleRate, uint BlockSize) {
    this->FrequencyBands = FrequencyBands;
	this->SampleRate = SampleRate;
	this->BlockSize = BlockSize;

    Div = new unsigned int[FrequencyBands];
	BandFrequency = new unsigned int[FrequencyBands+1];
	NyqistFrequency = new unsigned int[FrequencyBands];
	Output = new int[FrequencyBands];
	signal_lowfreq = new int[BlockSize];

    // TODO: initialize everything decently to avoid division by 0 errors and other problems
    for (uint k = 0; k < FrequencyBands; ++k) {
	    Div[k] = 1;		
    }

	amplitude = 1024;
    lowFreqBound = 20;
    highFreqBound = SampleRate >> 1; // Nyqist frequency

	calculateParameters(lowFreqBound, highFreqBound);
}

void CQT::calculateParameters(uint minFreq, uint maxFreq) {
	float base = 1.0091;
	minFreq = powf(base, minFreq);
	maxFreq = powf(base, maxFreq);

	if (minFreq < lowFreqBound) {
	    minFreq = lowFreqBound;
	}
	if (maxFreq > highFreqBound) {
	    maxFreq = highFreqBound;
	}

	// at least one octave will always be displayed.
	if (2 * minFreq > maxFreq) {	
	    if (2 * minFreq > highFreqBound) {
			maxFreq = highFreqBound;
			minFreq = highFreqBound / 2;
	    } else {
			maxFreq = 2 * minFreq;
	    }
	}

	// low frequencies must differ at least 60, so that the frequency width of each bucket is at least 2Hz. This prevents the number of samples needed to be larger than MAXSAMPLESIZE
	if (maxFreq - minFreq < 60) {	
	    maxFreq = minFreq + 60;
	}

    this->minFreq = minFreq;
    this->maxFreq = maxFreq;
    
	unsigned int Fmin = minFreq;
	unsigned int Fmax = maxFreq;

    // Calculating twoPiQ
    float nn = powf(2, log(Fmax / (double) Fmin) / log(2) / FrequencyBands);
    twoPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

    // Calculating the border frequencies...
    for (int i = 0; i < FrequencyBands + 1; ++i) {
		BandFrequency[i] = (Fmin * powf(nn, i) + Fmax / powf(nn, FrequencyBands - i)) / 2;
    }

    // Calculating the index until which the signal_lowfreq samples should be used
    lowfreq_endIndex = 0;
    while (SampleRate / (BandFrequency[lowfreq_endIndex + 1] - BandFrequency[lowfreq_endIndex]) >= BlockSize && lowfreq_endIndex < FrequencyBands) {
		++lowfreq_endIndex;
    }
    uint samplesLeft = BlockSize;

	uint i;
    // Calculating sample frequency dividers...
    for (i = FrequencyBands; i > lowfreq_endIndex; --i) {
		Div[i - 1] = 1 + SampleRate / ((BandFrequency[i] - BandFrequency[i - 1]) * samplesLeft / i);
		NyqistFrequency[i - 1] = SampleRate / (BandFrequency[i] - BandFrequency[i - 1]) / Div[i - 1];
		samplesLeft -= NyqistFrequency[i - 1];
    }
    for (; i > 0; --i) {
		Div[i - 1] = 1 + SampleRate / LOWFREQDIV / ((BandFrequency[i] - BandFrequency[i - 1]) * samplesLeft / i);
		NyqistFrequency[i - 1] = SampleRate / LOWFREQDIV / (BandFrequency[i] - BandFrequency[i - 1]) / Div[i - 1];
		samplesLeft -= NyqistFrequency[i - 1];
    }

	for (i=0;i<FrequencyBands;i++) {
		Serial.printf("%d Div=%d NyqistFreq=%d BandFrequency=%d\n",i , Div[i], NyqistFrequency[i], BandFrequency[i]);
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

fixedpoint CQT::approxCos(fixedpoint in) {
    return approxSin((int2PI >> 2) - in);
}


fixedpoint CQT::hamming(int m, int k) {
    return ALPHA - (BETA * approxCos(int2PI * m / NyqistFrequency[k]) >> PRECISION);
}

int* CQT::calculate(int32_t *signal) {
    unsigned int k, i, indx;
    int windowed, angle;
    float real_f, imag_f;
    fixedpoint real, imag;
	
	for (uint nrInterrupts=0;nrInterrupts<8;nrInterrupts++) {
		for (k = 0; k < lowfreq_endIndex; ++k) {
			indx = nrInterrupts % BlockSize - 1 + 8 * BlockSize;
			real = ALPHA - (BETA * signal_lowfreq[indx % BlockSize] >> PRECISION);
			imag = 0;
			for (i = 1; i < NyqistFrequency[k]; ++i) {
				windowed = hamming(i, k) * signal_lowfreq[(indx - i * Div[k]) % BlockSize];
				angle = twoPiQ * i / NyqistFrequency[k];
				real += windowed * approxCos(angle) >> PRECISION;
				imag += windowed * approxSin(angle) >> PRECISION;
			}

			real_f = real / (float) SCALE;
			imag_f = imag / (float) SCALE;
			BandFrequency[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NyqistFrequency[k] + 0.1) * amplitude / 32;
		}
		for (; k < FrequencyBands; ++k) {
			indx = nrInterrupts % BlockSize - 1 + 8 * BlockSize;
			real = ALPHA - (BETA * signal[indx % BlockSize] >> PRECISION);
			imag = 0;
			for (i = 1; i < NyqistFrequency[k]; ++i) {
				windowed = hamming(i, k) * signal[(indx - i * Div[k]) % BlockSize];
				angle = twoPiQ * i / NyqistFrequency[k];
				real += windowed * approxCos(angle) >> PRECISION;
				imag += windowed * approxSin(angle) >> PRECISION;
			}
			real_f = real / (float) SCALE;
			imag_f = imag / (float) SCALE;
			Output[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NyqistFrequency[k] + 0.1) * amplitude / 32;
		}

		return Output;
	}
}


